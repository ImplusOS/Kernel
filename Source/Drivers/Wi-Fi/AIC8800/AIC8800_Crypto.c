#include "AIC8800_Crypto.h"
#include "AIC8800.h"

#include <stddef.h>

/*
 * AIC8800_Crypto.c -- see AIC8800_Crypto.h. All of this is straight
 * FIPS 197 / FIPS 180-4 / RFC 2104 / RFC 2898; no vendor tables are
 * involved.
 */

/* ============================ SHA-1 ============================ */

static uint32_t aic_rol32(uint32_t x, uint32_t n)
{
    return (x << n) | (x >> (32u - n));
}

static void aic_sha1_block(uint32_t h[5], const uint8_t block[64])
{
    uint32_t w[80];
    for (uint32_t i = 0u; i < 16u; ++i) {
        w[i] = ((uint32_t)block[i * 4u] << 24) |
               ((uint32_t)block[i * 4u + 1u] << 16) |
               ((uint32_t)block[i * 4u + 2u] << 8) |
               ((uint32_t)block[i * 4u + 3u]);
    }
    for (uint32_t i = 16u; i < 80u; ++i) {
        w[i] = aic_rol32(w[i - 3u] ^ w[i - 8u] ^ w[i - 14u] ^ w[i - 16u], 1u);
    }

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (uint32_t i = 0u; i < 80u; ++i) {
        uint32_t f, k;
        if (i < 20u) {
            f = (b & c) | (~b & d);
            k = 0x5A827999u;
        } else if (i < 40u) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60u) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6u;
        }
        uint32_t tmp = aic_rol32(a, 5u) + f + e + k + w[i];
        e = d;
        d = c;
        c = aic_rol32(b, 30u);
        b = a;
        a = tmp;
    }

    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

void aic_sha1_init(aic_sha1_ctx_t *ctx)
{
    ctx->h[0] = 0x67452301u;
    ctx->h[1] = 0xEFCDAB89u;
    ctx->h[2] = 0x98BADCFEu;
    ctx->h[3] = 0x10325476u;
    ctx->h[4] = 0xC3D2E1F0u;
    ctx->total_len = 0u;
    ctx->block_len = 0u;
}

void aic_sha1_update(aic_sha1_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    ctx->total_len += len;

    while (len > 0u) {
        uint32_t space = 64u - ctx->block_len;
        uint32_t take = (len < space) ? len : space;
        for (uint32_t i = 0u; i < take; ++i) {
            ctx->block[ctx->block_len + i] = data[i];
        }
        ctx->block_len += take;
        data += take;
        len -= take;

        if (ctx->block_len == 64u) {
            aic_sha1_block(ctx->h, ctx->block);
            ctx->block_len = 0u;
        }
    }
}

void aic_sha1_final(aic_sha1_ctx_t *ctx, uint8_t out[20])
{
    uint64_t bit_len = ctx->total_len * 8u;

    /* Padding: 0x80, then zeros, then the 64-bit big-endian length. */
    uint8_t pad = 0x80u;
    aic_sha1_update(ctx, &pad, 1u);
    uint8_t zero = 0u;
    while (ctx->block_len != 56u) {
        aic_sha1_update(ctx, &zero, 1u);
    }
    uint8_t lenbuf[8];
    for (uint32_t i = 0u; i < 8u; ++i) {
        lenbuf[i] = (uint8_t)(bit_len >> (56u - i * 8u));
    }
    aic_sha1_update(ctx, lenbuf, 8u);

    for (uint32_t i = 0u; i < 20u; ++i) {
        out[i] = (uint8_t)(ctx->h[i / 4u] >> (24u - (i % 4u) * 8u));
    }
}

void aic_hmac_sha1(const uint8_t *key, uint32_t key_len,
                   const uint8_t *data, uint32_t data_len,
                   uint8_t out[20])
{
    uint8_t k[64];
    uint8_t ipad[64];
    uint8_t opad[64];

    for (uint32_t i = 0u; i < 64u; ++i) {
        k[i] = 0u;
    }
    if (key_len > 64u) {
        aic_sha1_ctx_t kctx;
        aic_sha1_init(&kctx);
        aic_sha1_update(&kctx, key, key_len);
        aic_sha1_final(&kctx, k); /* first 20 bytes, rest stay zero */
    } else {
        for (uint32_t i = 0u; i < key_len; ++i) {
            k[i] = key[i];
        }
    }
    for (uint32_t i = 0u; i < 64u; ++i) {
        ipad[i] = (uint8_t)(k[i] ^ 0x36u);
        opad[i] = (uint8_t)(k[i] ^ 0x5Cu);
    }

    uint8_t inner[20];
    aic_sha1_ctx_t ctx;
    aic_sha1_init(&ctx);
    aic_sha1_update(&ctx, ipad, 64u);
    aic_sha1_update(&ctx, data, data_len);
    aic_sha1_final(&ctx, inner);

    aic_sha1_init(&ctx);
    aic_sha1_update(&ctx, opad, 64u);
    aic_sha1_update(&ctx, inner, 20u);
    aic_sha1_final(&ctx, out);
}

void aic_pbkdf2_sha1(const uint8_t *pass, uint32_t pass_len,
                     const uint8_t *salt, uint32_t salt_len,
                     uint32_t iterations,
                     uint8_t *out, uint32_t out_len)
{
    uint32_t done = 0u;
    uint32_t block_index = 1u;

    while (done < out_len) {
        /* U1 = HMAC(pass, salt || INT_BE32(block_index)) */
        uint8_t u[20];
        uint8_t t[20];

        {
            /* HMAC with salt||INT_BE32(i) as the message: build it in a
             * scratch buffer (salt is the SSID here, at most 32 bytes,
             * so this is comfortably small). */
            uint8_t msg[40];
            uint32_t msg_len;
            if (salt_len > 36u) {
                return; /* not reachable for WPA2: salt is the 32-byte SSID */
            }
            msg_len = salt_len + 4u;
            for (uint32_t i = 0u; i < salt_len; ++i) {
                msg[i] = salt[i];
            }
            msg[salt_len + 0u] = (uint8_t)(block_index >> 24);
            msg[salt_len + 1u] = (uint8_t)(block_index >> 16);
            msg[salt_len + 2u] = (uint8_t)(block_index >> 8);
            msg[salt_len + 3u] = (uint8_t)(block_index);
            aic_hmac_sha1(pass, pass_len, msg, msg_len, u);
        }
        for (uint32_t i = 0u; i < 20u; ++i) {
            t[i] = u[i];
        }

        for (uint32_t iter = 1u; iter < iterations; ++iter) {
            aic_hmac_sha1(pass, pass_len, u, 20u, u);
            for (uint32_t i = 0u; i < 20u; ++i) {
                t[i] ^= u[i];
            }
        }

        uint32_t take = out_len - done;
        if (take > 20u) {
            take = 20u;
        }
        for (uint32_t i = 0u; i < take; ++i) {
            out[done + i] = t[i];
        }
        done += take;
        block_index++;
    }
}

/* ============================ AES-128 ============================ */

static const uint8_t aic_aes_sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};

static const uint8_t aic_aes_inv_sbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
};

static const uint8_t aic_aes_rcon[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

static uint8_t aic_gmul(uint8_t a, uint8_t b)
{
    uint8_t p = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) {
        if ((b & 1u) != 0u) {
            p ^= a;
        }
        uint8_t hi = (uint8_t)(a & 0x80u);
        a = (uint8_t)(a << 1);
        if (hi != 0u) {
            a ^= 0x1bu;
        }
        b = (uint8_t)(b >> 1);
    }
    return p;
}

void aic_aes128_init(aic_aes128_ctx_t *ctx, const uint8_t key[16])
{
    for (uint32_t i = 0u; i < 16u; ++i) {
        ctx->round_keys[i] = key[i];
    }
    for (uint32_t i = 16u; i < 176u; i += 4u) {
        uint8_t t[4];
        for (uint32_t j = 0u; j < 4u; ++j) {
            t[j] = ctx->round_keys[i - 4u + j];
        }
        if ((i % 16u) == 0u) {
            /* RotWord + SubWord + Rcon */
            uint8_t tmp = t[0];
            t[0] = (uint8_t)(aic_aes_sbox[t[1]] ^ aic_aes_rcon[i / 16u]);
            t[1] = aic_aes_sbox[t[2]];
            t[2] = aic_aes_sbox[t[3]];
            t[3] = aic_aes_sbox[tmp];
        }
        for (uint32_t j = 0u; j < 4u; ++j) {
            ctx->round_keys[i + j] =
                (uint8_t)(ctx->round_keys[i - 16u + j] ^ t[j]);
        }
    }
}

static void aic_aes_add_round_key(uint8_t state[16], const uint8_t *rk)
{
    for (uint32_t i = 0u; i < 16u; ++i) {
        state[i] ^= rk[i];
    }
}

static void aic_aes_sub_bytes(uint8_t state[16])
{
    for (uint32_t i = 0u; i < 16u; ++i) {
        state[i] = aic_aes_sbox[state[i]];
    }
}

static void aic_aes_inv_sub_bytes(uint8_t state[16])
{
    for (uint32_t i = 0u; i < 16u; ++i) {
        state[i] = aic_aes_inv_sbox[state[i]];
    }
}

/* State bytes are stored column-major (state[r + 4c]). */
static void aic_aes_shift_rows(uint8_t s[16])
{
    uint8_t t;

    /* row 1: <<< 1 */
    t = s[1];
    s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
    /* row 2: <<< 2 */
    t = s[2]; s[2] = s[10]; s[10] = t;
    t = s[6]; s[6] = s[14]; s[14] = t;
    /* row 3: <<< 3 */
    t = s[15];
    s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;
}

static void aic_aes_inv_shift_rows(uint8_t s[16])
{
    uint8_t t;

    /* row 1: >>> 1 */
    t = s[13];
    s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
    /* row 2: >>> 2 */
    t = s[2]; s[2] = s[10]; s[10] = t;
    t = s[6]; s[6] = s[14]; s[14] = t;
    /* row 3: >>> 3 */
    t = s[3];
    s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;
}

static void aic_aes_mix_columns(uint8_t s[16])
{
    for (uint32_t c = 0u; c < 4u; ++c) {
        uint8_t *col = &s[c * 4u];
        uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        col[0] = (uint8_t)(aic_gmul(a0, 2u) ^ aic_gmul(a1, 3u) ^ a2 ^ a3);
        col[1] = (uint8_t)(a0 ^ aic_gmul(a1, 2u) ^ aic_gmul(a2, 3u) ^ a3);
        col[2] = (uint8_t)(a0 ^ a1 ^ aic_gmul(a2, 2u) ^ aic_gmul(a3, 3u));
        col[3] = (uint8_t)(aic_gmul(a0, 3u) ^ a1 ^ a2 ^ aic_gmul(a3, 2u));
    }
}

static void aic_aes_inv_mix_columns(uint8_t s[16])
{
    for (uint32_t c = 0u; c < 4u; ++c) {
        uint8_t *col = &s[c * 4u];
        uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        col[0] = (uint8_t)(aic_gmul(a0, 14u) ^ aic_gmul(a1, 11u) ^
                           aic_gmul(a2, 13u) ^ aic_gmul(a3, 9u));
        col[1] = (uint8_t)(aic_gmul(a0, 9u) ^ aic_gmul(a1, 14u) ^
                           aic_gmul(a2, 11u) ^ aic_gmul(a3, 13u));
        col[2] = (uint8_t)(aic_gmul(a0, 13u) ^ aic_gmul(a1, 9u) ^
                           aic_gmul(a2, 14u) ^ aic_gmul(a3, 11u));
        col[3] = (uint8_t)(aic_gmul(a0, 11u) ^ aic_gmul(a1, 13u) ^
                           aic_gmul(a2, 9u) ^ aic_gmul(a3, 14u));
    }
}

void aic_aes128_encrypt(const aic_aes128_ctx_t *ctx, const uint8_t in[16], uint8_t out[16])
{
    uint8_t state[16];
    for (uint32_t i = 0u; i < 16u; ++i) {
        state[i] = in[i];
    }

    aic_aes_add_round_key(state, &ctx->round_keys[0]);
    for (uint32_t round = 1u; round < 10u; ++round) {
        aic_aes_sub_bytes(state);
        aic_aes_shift_rows(state);
        aic_aes_mix_columns(state);
        aic_aes_add_round_key(state, &ctx->round_keys[round * 16u]);
    }
    aic_aes_sub_bytes(state);
    aic_aes_shift_rows(state);
    aic_aes_add_round_key(state, &ctx->round_keys[160u]);

    for (uint32_t i = 0u; i < 16u; ++i) {
        out[i] = state[i];
    }
}

void aic_aes128_decrypt(const aic_aes128_ctx_t *ctx, const uint8_t in[16], uint8_t out[16])
{
    uint8_t state[16];
    for (uint32_t i = 0u; i < 16u; ++i) {
        state[i] = in[i];
    }

    aic_aes_add_round_key(state, &ctx->round_keys[160u]);
    for (uint32_t round = 9u; round > 0u; --round) {
        aic_aes_inv_shift_rows(state);
        aic_aes_inv_sub_bytes(state);
        aic_aes_add_round_key(state, &ctx->round_keys[round * 16u]);
        aic_aes_inv_mix_columns(state);
    }
    aic_aes_inv_shift_rows(state);
    aic_aes_inv_sub_bytes(state);
    aic_aes_add_round_key(state, &ctx->round_keys[0]);

    for (uint32_t i = 0u; i < 16u; ++i) {
        out[i] = state[i];
    }
}

bool aic_aes128_kw_unwrap(const uint8_t kek[16],
                          const uint8_t *wrapped, uint32_t wrapped_len,
                          uint8_t *out, uint32_t out_cap, uint32_t *out_len)
{
    static const uint8_t iv[8] = { 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6 };
    uint8_t a[8];
    uint8_t r[8 * 8]; /* up to 8 64-bit blocks: 64 bytes of key material */
    uint32_t n;

    if (wrapped_len < 24u || (wrapped_len % 8u) != 0u) {
        return false;
    }
    n = (wrapped_len / 8u) - 1u;
    if (n > 8u) {
        return false;
    }
    if ((n * 8u) > out_cap) {
        return false;
    }

    aic_aes128_ctx_t ctx;
    aic_aes128_init(&ctx, kek);

    for (uint32_t i = 0u; i < 8u; ++i) {
        a[i] = wrapped[i];
    }
    for (uint32_t i = 0u; i < n * 8u; ++i) {
        r[i] = wrapped[8u + i];
    }

    for (uint32_t j = 6u; j > 0u; --j) {
        for (uint32_t i = n; i > 0u; --i) {
            uint32_t t = j * n + i;
            uint8_t block_in[16];
            uint8_t block_out[16];

            for (uint32_t k = 0u; k < 8u; ++k) {
                block_in[k] = a[k];
            }
            /* XOR the 64-bit counter in (t < 2^32, so the top 4 bytes
             * are untouched). */
            block_in[4] ^= (uint8_t)(t >> 24);
            block_in[5] ^= (uint8_t)(t >> 16);
            block_in[6] ^= (uint8_t)(t >> 8);
            block_in[7] ^= (uint8_t)(t);
            for (uint32_t k = 0u; k < 8u; ++k) {
                block_in[8u + k] = r[(i - 1u) * 8u + k];
            }

            aic_aes128_decrypt(&ctx, block_in, block_out);

            for (uint32_t k = 0u; k < 8u; ++k) {
                a[k] = block_out[k];
                r[(i - 1u) * 8u + k] = block_out[8u + k];
            }
        }
    }

    for (uint32_t i = 0u; i < 8u; ++i) {
        if (a[i] != iv[i]) {
            return false;
        }
    }

    for (uint32_t i = 0u; i < n * 8u; ++i) {
        out[i] = r[i];
    }
    *out_len = n * 8u;
    return true;
}

/* ============================ randomness ============================ */

static uint64_t g_aic_rng_state = 0u;

static uint64_t aic_rng_next(void)
{
    uint64_t x = g_aic_rng_state;
    if (x == 0u) {
        x = 0x9E3779B97F4A7C15u;
    }
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    g_aic_rng_state = x;
    return x * 0x2545F4914F6CDD1Du;
}

void aic_random_bytes(uint8_t *out, uint32_t len)
{
    static uint32_t calls = 0u;

    if (g_aic_api != NULL) {
        uint64_t ticks = (g_aic_api->timer_ticks != NULL) ? g_aic_api->timer_ticks() : 0u;
        uint32_t hz = (g_aic_api->timer_hz != NULL) ? g_aic_api->timer_hz() : 0u;
        g_aic_rng_state ^= (ticks * 0x9E3779B97F4A7C15u) ^ (uint64_t)hz;
    }
    g_aic_rng_state ^= (uint64_t)(++calls) * 0xBF58476D1CE4E5B9u;

    for (uint32_t i = 0u; i < len; i += 8u) {
        uint64_t v = aic_rng_next();
        for (uint32_t j = 0u; j < 8u && (i + j) < len; ++j) {
            out[i + j] = (uint8_t)(v >> (j * 8u));
        }
    }
}
