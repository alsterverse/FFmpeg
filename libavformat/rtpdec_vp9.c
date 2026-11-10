/*
 * RTP parser for VP9 payload format (RFC 9628) - experimental
 * Copyright (c) 2015 Thomas Volkert <thomas@homer-conferencing.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/intreadwrite.h"

#include "avio_internal.h"
#include "rtpdec_formats.h"

#define RTP_VP9_DESC_REQUIRED_SIZE 1
#define RTP_VP9_MAX_P_DIFF         3

struct PayloadContext {
    AVIOContext *buf;
    uint32_t     timestamp;
    int          is_keyframe;
};

static av_cold int vp9_init(AVFormatContext *ctx, int st_index,
                            PayloadContext *data)
{
    av_log(ctx, AV_LOG_WARNING,
           "RTP/VP9 support is still experimental\n");

    return 0;
}

static int vp9_handle_packet(AVFormatContext *ctx, PayloadContext *rtp_vp9_ctx,
                             AVStream *st, AVPacket *pkt, uint32_t *timestamp,
                             const uint8_t *buf, int len, uint16_t seq,
                             int flags)
{
    int has_pic_id, inter_pic_predicted, has_layer_idc, flexible_mode;
    int start_of_frame, end_of_frame, has_ss_data;
    av_unused int pic_id = 0, layer_temporal = 0;
    int layer_spatial = 0;
    int rtp_m;
    int res = 0;

    /* drop data of previous packets in case of non-continuous (lossy) packet stream */
    if (rtp_vp9_ctx->buf && rtp_vp9_ctx->timestamp != *timestamp)
        ffio_free_dyn_buf(&rtp_vp9_ctx->buf);

    /* sanity check for size of input packet: 1 byte payload at least */
    if (len < RTP_VP9_DESC_REQUIRED_SIZE + 1) {
        av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet, got %d bytes\n", len);
        return AVERROR_INVALIDDATA;
    }

    /*
     *     decode the required VP9 payload descriptor according to section 4.2 of RFC 9628:
     *
     *      0 1 2 3 4 5 6 7
     *     +-+-+-+-+-+-+-+-+
     *     |I|P|L|F|B|E|V|Z| (REQUIRED)
     *     +-+-+-+-+-+-+-+-+
     *
     *     I: Picture ID (PID) present
     *     P: Inter-picture predicted frame
     *     L: Layer indices present
     *     F: Flexible mode
     *     B: Start of a frame
     *     E: End of a frame
     *     V: Scalability Structure (SS) data present
     *     Z: Not a reference frame for upper spatial layers
     */
    has_pic_id          = !!(buf[0] & 0x80);
    inter_pic_predicted = !!(buf[0] & 0x40);
    has_layer_idc       = !!(buf[0] & 0x20);
    flexible_mode       = !!(buf[0] & 0x10);
    start_of_frame      = !!(buf[0] & 0x08);
    end_of_frame        = !!(buf[0] & 0x04);
    has_ss_data         = !!(buf[0] & 0x02);

    /* F must be zero and is ignored by receivers if I is zero */
    if (!has_pic_id)
        flexible_mode = 0;

    rtp_m = !!(flags & RTP_FLAG_MARKER);

    /* sanity check for markers: the RTP M marker ends a picture, so it
     * can only be set on the last packet of a frame (E) */
    if (rtp_m && !end_of_frame) {
        av_log(ctx, AV_LOG_ERROR, "Invalid combination of E and M marker (E unset, M set)\n");
        return AVERROR_INVALIDDATA;
    }

    /* pass the extensions field */
    buf += RTP_VP9_DESC_REQUIRED_SIZE;
    len -= RTP_VP9_DESC_REQUIRED_SIZE;

    /*
     *         decode the 1-byte/2-byte picture ID:
     *
     *          0 1 2 3 4 5 6 7
     *         +-+-+-+-+-+-+-+-+
     *   I:    |M|PICTURE ID   | (RECOMMENDED)
     *         +-+-+-+-+-+-+-+-+
     *   M:    | EXTENDED PID  | (RECOMMENDED)
     *         +-+-+-+-+-+-+-+-+
     *
     *   M: The most significant bit of the first octet is an extension flag.
     *   PictureID:  7 or 15 bits, depending on the M bit.
     */
    if (has_pic_id) {
        /* check for 1-byte or 2-byte picture index */
        if (buf[0] & 0x80) {
            if (len < 2) {
                av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                return AVERROR_INVALIDDATA;
            }
            pic_id = AV_RB16(buf) & 0x7fff;
            buf += 2;
            len -= 2;
        } else {
            pic_id = buf[0] & 0x7f;
            buf++;
            len--;
        }
    }

    /*
     *         decode layer indices
     *
     *          0 1 2 3 4 5 6 7
     *         +-+-+-+-+-+-+-+-+
     *   L:    | TID |U| SID |D| (CONDITIONALLY RECOMMENDED)
     *         +-+-+-+-+-+-+-+-+
     *         |   TL0PICIDX   | (CONDITIONALLY REQUIRED)
     *         +-+-+-+-+-+-+-+-+
     *
     *   TID: The temporal layer ID.
     *   U: Switching up point.
     *   SID: The spatial layer ID.
     *   D: Inter-layer dependency is used.
     *   TL0PICIDX: Temporal layer 0 picture index, only present in
     *              non-flexible mode (F = 0).
     */
    if (has_layer_idc) {
        int layer_idc_size = flexible_mode ? 1 : 2;
        if (len < layer_idc_size) {
            av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
            return AVERROR_INVALIDDATA;
        }
        layer_temporal = buf[0] >> 5;
        layer_spatial  = (buf[0] >> 1) & 0x07;
        buf += layer_idc_size;
        len -= layer_idc_size;

        if (layer_spatial > 0) {
            avpriv_report_missing_feature(ctx, "VP9 spatial layers");
            return AVERROR_PATCHWELCOME;
        }
    }

    /*
     *         decode the reference indices
     *
     *          0 1 2 3 4 5 6 7
     *         +-+-+-+-+-+-+-+-+                             -\
     *   P,F:  | P_DIFF      |N| (CONDITIONALLY REQUIRED)    - up to 3 times
     *         +-+-+-+-+-+-+-+-+                             -/
     *
     *   P_DIFF: The relative Picture ID referred to by this frame.
     *   N: 1 if there is another P_DIFF following this one.
     */
    if (inter_pic_predicted && flexible_mode) {
        int p_diff_count = 0, more_p_diff;
        do {
            if (len < 1) {
                av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                return AVERROR_INVALIDDATA;
            }
            if (++p_diff_count > RTP_VP9_MAX_P_DIFF) {
                av_log(ctx, AV_LOG_ERROR, "Too many RTP/VP9 reference indices\n");
                return AVERROR_INVALIDDATA;
            }
            more_p_diff = buf[0] & 0x01;
            buf++;
            len--;
        } while (more_p_diff);
    }

    /*
     *         decode the scalability structure (SS)
     *
     *          0 1 2 3 4 5 6 7
     *         +-+-+-+-+-+-+-+-+
     *   V:    | N_S |Y|G|-|-|-|
     *         +-+-+-+-+-+-+-+-+              -\
     *   Y:    |     WIDTH     | (OPTIONAL)    .
     *         +               +               .
     *         |               | (OPTIONAL)    .
     *         +-+-+-+-+-+-+-+-+               . - N_S + 1 times
     *         |     HEIGHT    | (OPTIONAL)    .
     *         +               +               .
     *         |               | (OPTIONAL)    .
     *         +-+-+-+-+-+-+-+-+              -/
     *   G:    |      N_G      | (OPTIONAL)
     *         +-+-+-+-+-+-+-+-+                            -\
     *   N_G:  | TID |U| R |-|-| (OPTIONAL)                 .
     *         +-+-+-+-+-+-+-+-+              -\            . - N_G times
     *         |    P_DIFF     | (OPTIONAL)    . - R times  .
     *         +-+-+-+-+-+-+-+-+              -/            -/
     *
     *   N_S: Number of spatial layers minus 1.
     *   Y: Each spatial layer's frame resolution is present.
     *   G: Picture group (PG) description is present.
     *   N_G: Number of pictures in the PG.
     *   R: Number of P_DIFF fields of a picture in the PG.
     */
    if (has_ss_data) {
        int n_s, y, g, i;
        if (len < 1) {
            av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
            return AVERROR_INVALIDDATA;
        }
        n_s = buf[0] >> 5;
        y = !!(buf[0] & 0x10);
        g = !!(buf[0] & 0x08);
        buf++;
        len--;
        if (n_s > 0) {
            avpriv_report_missing_feature(ctx, "VP9 scalability structure with multiple layers");
            return AVERROR_PATCHWELCOME;
        }
        if (y) {
            if (len < 4 * (n_s + 1)) {
                av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                return AVERROR_INVALIDDATA;
            }
            for (i = 0; i < n_s + 1; i++) {
                av_unused int w, h;
                w = AV_RB16(buf);
                h = AV_RB16(buf + 2);
                buf += 4;
                len -= 4;
            }
        }
        if (g) {
            int n_g;
            if (len < 1) {
                av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                return AVERROR_INVALIDDATA;
            }
            n_g = buf[0];
            buf++;
            len--;
            for (i = 0; i < n_g; i++) {
                av_unused int t, u, r, j;
                if (len < 1) {
                    av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                    return AVERROR_INVALIDDATA;
                }
                t = buf[0] >> 5;
                u = !!(buf[0] & 0x10);
                r = (buf[0] >> 2) & 0x03;
                buf++;
                len--;
                if (len < r) {
                    av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
                    return AVERROR_INVALIDDATA;
                }
                for (j = 0; j < r; j++) {
                    av_unused int p_diff = buf[0];
                    buf++;
                    len--;
                }
            }
        }
    }

    /* sanity check: 1 byte payload as minimum */
    if (len < 1) {
        av_log(ctx, AV_LOG_ERROR, "Too short RTP/VP9 packet\n");
        return AVERROR_INVALIDDATA;
    }

    /* start frame buffering with new dynamic buffer */
    if (!rtp_vp9_ctx->buf) {
        /* sanity check: a new frame should have started */
        if (start_of_frame) {
            res = avio_open_dyn_buf(&rtp_vp9_ctx->buf);
            if (res < 0)
                return res;
            /* update the timestamp in the frame packet with the one from the RTP packet */
            rtp_vp9_ctx->timestamp = *timestamp;
            /* the first packet of a key picture has P = 0, B = 1 and
             * SID = 0 (only the base spatial layer is supported) */
            rtp_vp9_ctx->is_keyframe = !inter_pic_predicted;
        } else {
            /* frame not started yet, need more packets */
            return AVERROR(EAGAIN);
        }
    }

    /* write the fragment to the dyn. buffer */
    avio_write(rtp_vp9_ctx->buf, buf, len);

    /* do we need more fragments? */
    if (!end_of_frame)
        return AVERROR(EAGAIN);

    /* close frame buffering and create resulting A/V packet */
    res = ff_rtp_finalize_packet(pkt, &rtp_vp9_ctx->buf, st->index);
    if (res < 0)
        return res;

    if (rtp_vp9_ctx->is_keyframe)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

static void vp9_close_context(PayloadContext *vp9)
{
    ffio_free_dyn_buf(&vp9->buf);
}

const RTPDynamicProtocolHandler ff_vp9_dynamic_handler = {
    .enc_name         = "VP9",
    .codec_type       = AVMEDIA_TYPE_VIDEO,
    .codec_id         = AV_CODEC_ID_VP9,
    .priv_data_size   = sizeof(PayloadContext),
    .init             = vp9_init,
    .close            = vp9_close_context,
    .parse_packet     = vp9_handle_packet
};
