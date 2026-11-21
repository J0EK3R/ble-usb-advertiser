/**
 * @file bc2_scanfilter.h
 * @brief Duplicate filter and rate budget for scan reports, plus the address type cache used by
 *        CONNECT with addr_type = 0xFF (auto).
 *
 * Pure C11, not thread safe (used from the NimBLE host task / one adapter lock only).
 *
 * Why a filter: a busy 2.4 GHz environment delivers hundreds of advertising PDUs per second, most
 * of them repeats. The stick reports a PDU only when its (address, address type, scan-response
 * flag, data hash) combination is new, changed, or was last reported more than `refresh_ms` ago,
 * and never more than BC2_SCANFILTER_BUDGET reports per second, so neither USB nor the host can be
 * flooded. RSSI is not part of the key (it changes with every PDU).
 */
#ifndef BC2_SCANFILTER_H
#define BC2_SCANFILTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BC2_SCANFILTER_ENTRIES 48u     /**< remembered (address, type, scan-rsp) combinations */
#define BC2_SCANFILTER_BUDGET 150u     /**< maximum reports per second */
#define BC2_ADDRCACHE_ENTRIES 64u      /**< remembered address types */

/** One remembered advertiser. */
typedef struct {
    bool used;
    bool scan_rsp;                     /**< entry belongs to scan responses of the address */
    uint8_t addr[6];                   /**< display order */
    uint8_t type;                      /**< 0 public, 1 random */
    uint32_t hash;                     /**< FNV-1a of the last reported data */
    uint32_t last_ms;                  /**< time of the last report */
} bc2_sf_entry_t;

/** Filter state. Initialise with bc2_scanfilter_init(). */
typedef struct {
    bc2_sf_entry_t entries[BC2_SCANFILTER_ENTRIES];
    uint16_t refresh_ms;               /**< re-report period of unchanged advertisers */
    bool report_all;                   /**< SCAN_START "report duplicates": no dedupe, budget only */
    uint32_t window_start_ms;          /**< start of the current 1 s budget window */
    uint16_t window_count;             /**< reports accepted in the current window */
    uint32_t dropped_budget;           /**< reports refused by the budget (total) */
} bc2_scanfilter_t;

/** Verdict for one received PDU. */
typedef enum {
    BC2_SF_REPORT,                     /**< forward it */
    BC2_SF_DUPLICATE,                  /**< unchanged and recently reported: silently ignore */
    BC2_SF_OVER_BUDGET                 /**< new or changed, but the 150/s budget is used up: count as dropped */
} bc2_sf_verdict_t;

/** Clear the filter and set its parameters. */
void bc2_scanfilter_init(bc2_scanfilter_t *f, uint16_t refresh_ms, bool report_all);

/**
 * Decide whether a received advertising PDU is reported.
 * @param addr     advertiser address in display order
 * @param type     0 public, 1 random
 * @param scan_rsp true for a scan response PDU
 * @param data     raw AD bytes
 * @param now_ms   wrap-safe millisecond clock
 */
bc2_sf_verdict_t bc2_scanfilter_check(bc2_scanfilter_t *f, const uint8_t addr[6], uint8_t type,
                                      bool scan_rsp, const uint8_t *data, size_t len, uint32_t now_ms);

/** LRU cache from address to address type. */
typedef struct {
    struct {
        bool used;
        uint8_t addr[6];
        uint8_t type;
        uint32_t stamp;                /**< LRU counter value of the last use */
    } e[BC2_ADDRCACHE_ENTRIES];
    uint32_t clock;                    /**< increments on every put/get */
} bc2_addrcache_t;

void bc2_addrcache_init(bc2_addrcache_t *c);
/** Remember (or refresh) the type of an address; evicts the least recently used entry when full. */
void bc2_addrcache_put(bc2_addrcache_t *c, const uint8_t addr[6], uint8_t type);
/** Look up an address; refreshes its LRU position. @return false if unknown. */
bool bc2_addrcache_get(bc2_addrcache_t *c, const uint8_t addr[6], uint8_t *type);

#ifdef __cplusplus
}
#endif

#endif /* BC2_SCANFILTER_H */
