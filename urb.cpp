#include "urb.hpp"
#include "mgr.hpp"

#include <algorithm>

namespace ps3eye {

URBDesc::URBDesc() = default;

static void LIBUSB_CALL transfer_completed_callback(struct libusb_transfer* xfr);

static void LIBUSB_CALL transfer_completed_callback(struct libusb_transfer* xfr)
{
    URBDesc* urb = reinterpret_cast<URBDesc*>(xfr->user_data);
    enum libusb_transfer_status status = xfr->status;

    if (status != LIBUSB_TRANSFER_COMPLETED)
    {
        ps3eye_debug("transfer status %d\n", status);

        urb->transfer_canceled();

        if (status != LIBUSB_TRANSFER_CANCELLED)
        {
            urb->close_transfers();
        }
        return;
    }

    // debug("length:%u, actual_length:%u\n", xfr->length, xfr->actual_length);

    urb->pkt_scan(xfr->buffer, xfr->actual_length);

    if (libusb_submit_transfer(xfr) < 0)
    {
        ps3eye_debug("error re-submitting URB\n");
        urb->close_transfers();
    }
}

/*
 * look for an input transfer endpoint in an alternate setting
 * libusb_endpoint_descriptor
 */
static uint8_t find_ep(struct libusb_device* device)
{
    const struct libusb_interface_descriptor* altsetting = NULL;
    const struct libusb_endpoint_descriptor* ep;
    struct libusb_config_descriptor* config = NULL;
    int i;
    uint8_t ep_addr = 0;

    libusb_get_active_config_descriptor(device, &config);

    if (!config) return 0;

    for (i = 0; i < config->bNumInterfaces; i++)
    {
        altsetting = config->interface[i].altsetting;
        if (altsetting[0].bInterfaceNumber == 0)
        {
            break;
        }
    }

    if (!altsetting)
        return 0;

    for (i = 0; i < altsetting->bNumEndpoints; i++)
    {
        ep = &altsetting->endpoint[i];
        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK &&
            ep->wMaxPacketSize != 0)
        {
            ep_addr = ep->bEndpointAddress;
            break;
        }
    }

    libusb_free_config_descriptor(config);

    return ep_addr;
}

bool URBDesc::start_transfers(libusb_device_handle* handle, uint32_t curr_frame_size)
{
    // Initialize the frame queue
    frame_size = curr_frame_size;
    frame_queue = new FrameQueue(frame_size);

    // Initialize the current frame pointer to the start of the buffer; it
    // will be updated as frames are completed and pushed onto the frame queue
    cur_frame_start = frame_queue->GetFrameBufferStart();
    cur_frame_data_len = 0;

    // Find the bulk transfer endpoint
    uint8_t bulk_endpoint = find_ep(libusb_get_device(handle));
    libusb_clear_halt(handle, bulk_endpoint);

    // Allocate the transfer buffer
    transfer_buffer = (uint8_t*)malloc(TRANSFER_SIZE * NUM_TRANSFERS);
    memset(transfer_buffer, 0, TRANSFER_SIZE * NUM_TRANSFERS);

    int res = 0;
    for (int index = 0; index < NUM_TRANSFERS; ++index)
    {
        // Create & submit the transfer
        xfr[index] = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(xfr[index], handle, bulk_endpoint,
                                  transfer_buffer + index * TRANSFER_SIZE,
                                  TRANSFER_SIZE, transfer_completed_callback,
                                  reinterpret_cast<void*>(this), 0);

        res |= libusb_submit_transfer(xfr[index]);

        num_active_transfers++;
    }

    last_pts = 0;
    last_fid = 0;

    USBMgr::instance()->cameraStarted();

    return res == 0;
}

void URBDesc::close_transfers()
{
    std::unique_lock<std::mutex> lock(num_active_transfers_mutex);
    if (num_active_transfers == 0) return;

    // Cancel any pending transfers
    for (int index = 0; index < NUM_TRANSFERS; ++index)
    {
        libusb_cancel_transfer(xfr[index]);
    }

    // Wait for cancelation to finish
    num_active_transfers_condition.wait(lock, [this]() {
        return num_active_transfers == 0;
    });

    // Free completed transfers
    for (int index = 0; index < NUM_TRANSFERS; ++index)
    {
        libusb_free_transfer(xfr[index]);
        xfr[index] = nullptr;
    }

    USBMgr::instance()->cameraStopped();

    free(transfer_buffer);
    transfer_buffer = nullptr;

    delete frame_queue;
    frame_queue = nullptr;
}

void URBDesc::transfer_canceled()
{
    std::lock_guard<std::mutex> lock(num_active_transfers_mutex);
    --num_active_transfers;
    num_active_transfers_condition.notify_one();
}

void URBDesc::frame_add(enum gspca_packet_type packet_type, const uint8_t* data, int len)
{
    if (packet_type == FIRST_PACKET)
    {
        cur_frame_data_len = 0;
    }
    else
    {
        switch (last_packet_type) // ignore warning.
        {
        case DISCARD_PACKET:
            if (packet_type == LAST_PACKET)
            {
                last_packet_type = packet_type;
                cur_frame_data_len = 0;
            }
            return;
        case LAST_PACKET:
            return;
        default:
            break;
        }
    }

    /* append the packet to the frame buffer */
    if (len > 0)
    {
        if (cur_frame_data_len + (unsigned)len > frame_size)
        {
            packet_type = DISCARD_PACKET;
            cur_frame_data_len = 0;
        }
        else
        {
            memcpy(cur_frame_start + cur_frame_data_len, data, (unsigned)len);
            cur_frame_data_len += (unsigned)len;
        }
    }

    last_packet_type = packet_type;

    if (packet_type == LAST_PACKET)
    {
        cur_frame_data_len = 0;
        cur_frame_start = frame_queue->Enqueue();
        // debug("frame completed %d\n", frame_complete_ind);
    }
}

void URBDesc::pkt_scan(uint8_t* data, int len)
{
    uint32_t this_pts;
    uint16_t this_fid;
    int remaining_len = len;
    int payload_len;

    payload_len = 2048; // bulk type
    do
    {
        len = std::min(remaining_len, payload_len);

        /* Payloads are prefixed with a UVC-style header.  We
           consider a frame to start when the FID toggles, or the PTS
           changes.  A frame ends when EOF is set, and we've received
           the correct number of bytes. */

        /* Verify UVC header.  Header length is always 12 */
        if (data[0] != 12 || len < 12)
        {
            ps3eye_debug("bad header\n");
            goto discard;
        }

        /* Check errors */
        if (data[1] & UVC_STREAM_ERR)
        {
            ps3eye_debug("payload error\n");
            goto discard;
        }

        /* Extract PTS and FID */
        if (!(data[1] & UVC_STREAM_PTS))
        {
            ps3eye_debug("PTS not present\n");
            goto discard;
        }

        this_pts = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
        this_fid = !!(data[1] & UVC_STREAM_FID);

        /* If PTS or FID has changed, start a new frame. */
        if (this_pts != last_pts || this_fid != last_fid)
        {
            if (last_packet_type == INTER_PACKET)
            {
                /* The last frame was incomplete, so don't keep it or we
                 * will glitch */
                frame_add(DISCARD_PACKET, nullptr, 0);
            }
            last_pts = this_pts;
            last_fid = this_fid;
            frame_add(FIRST_PACKET, data + 12, len - 12);
        } /* If this packet is marked as EOF, end the frame */
        else if (data[1] & UVC_STREAM_EOF)
        {
            last_pts = 0;
            if (cur_frame_data_len + (unsigned)len - 12 != frame_size)
            {
                goto discard;
            }
            frame_add(LAST_PACKET, data + 12, len - 12);
        }
        else
        {
            /* Add the data from this payload */
            frame_add(INTER_PACKET, data + 12, len - 12);
        }

        /* Done this payload */
        goto scan_next;

discard:
        /* Discard data until a new frame starts. */
        frame_add(DISCARD_PACKET, nullptr, 0);
scan_next:
        remaining_len -= len;
        data += len;
    } while (remaining_len > 0);
}

} // namespace ps3eye
