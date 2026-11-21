/**
 * @file bc2_scanfilter.c
 * @brief Implementation of the scan report filter and the address type cache.
 */
#include "bc2_scanfilter.h"

#include <string.h>

/** FNV-1a 32 bit hash of the advertising data. */
static uint32_t fnv1a(const uint8_t *data, size_t len)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h = (h ^ data[i]) * 16777619u;
    }
    return h;
}

/** Milliseconds from @p then to @p now (wrap-safe for periods below 2^31 ms). */
static uint32_t age_ms(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

void bc2_scanfilter_init(bc2_scanfilter_t *f, uint16_t refresh_ms, bool report_all)
{
    memset(f, 0, sizeof(*f));
    f->refresh_ms = refresh_ms;
    f->report_all = report_all;
}

/** Find the entry of a combination, or NULL. */
static bc2_sf_entry_t *find_entry(bc2_scanfilter_t *f, const uint8_t addr[6], uint8_t type, bool scan_rsp)
{
    for (size_t i = 0; i < BC2_SCANFILTER_ENTRIES; i++) {
        bc2_sf_entry_t *e = &f->entries[i];
        if (e->used && e->type == type && e->scan_rsp == scan_rsp && memcmp(e->addr, addr, 6) == 0) {
            return e;
        }
    }
    return NULL;
}

/** Free slot, or when none is free the entry with the oldest last_ms (LRU eviction). */
static bc2_sf_entry_t *allocate_entry(bc2_scanfilter_t *f, uint32_t now_ms)
{
    bc2_sf_entry_t *oldest = &f->entries[0];
    for (size_t i = 0; i < BC2_SCANFILTER_ENTRIES; i++) {
        bc2_sf_entry_t *e = &f->entries[i];
        if (!e->used) {
            return e;
        }
        if (age_ms(now_ms, e->last_ms) > age_ms(now_ms, oldest->last_ms)) {
            oldest = e;
        }
    }
    return oldest;
}

/** Consume one unit of the per-second budget; false if it is used up. */
static bool take_budget(bc2_scanfilter_t *f, uint32_t now_ms)
{
    if (age_ms(now_ms, f->window_start_ms) >= 1000u) {      /* new 1 s window */
        f->window_start_ms = now_ms;
        f->window_count = 0;
    }
    if (f->window_count >= BC2_SCANFILTER_BUDGET) {
        return false;
    }
    f->window_count++;
    return true;
}

bc2_sf_verdict_t bc2_scanfilter_check(bc2_scanfilter_t *f, const uint8_t addr[6], uint8_t type,
                                      bool scan_rsp, const uint8_t *data, size_t len, uint32_t now_ms)
{
    const uint32_t hash = fnv1a(data, len);
    bc2_sf_entry_t *e = find_entry(f, addr, type, scan_rsp);
    const bool fresh = e != NULL && e->hash == hash && age_ms(now_ms, e->last_ms) < f->refresh_ms;
    if (fresh && !f->report_all) {
        return BC2_SF_DUPLICATE;
    }
    if (!take_budget(f, now_ms)) {
        f->dropped_budget++;
        return BC2_SF_OVER_BUDGET;       /* the entry stays unchanged, so the PDU is retried later */
    }
    if (e == NULL) {
        e = allocate_entry(f, now_ms);
        e->used = true;
        e->scan_rsp = scan_rsp;
        e->type = type;
        memcpy(e->addr, addr, 6);
    }
    e->hash = hash;
    e->last_ms = now_ms;
    return BC2_SF_REPORT;
}

/* ---- address type cache ---- */

void bc2_addrcache_init(bc2_addrcache_t *c)
{
    memset(c, 0, sizeof(*c));
}

void bc2_addrcache_put(bc2_addrcache_t *c, const uint8_t addr[6], uint8_t type)
{
    size_t victim = 0;           /* entry to (over)write: the known one, a free one or the LRU one */
    bool found = false;
    for (size_t i = 0; i < BC2_ADDRCACHE_ENTRIES && !found; i++) {
        if (c->e[i].used && memcmp(c->e[i].addr, addr, 6) == 0) {
            victim = i;
            found = true;                                    /* known address: update in place */
        }
    }
    for (size_t i = 0; i < BC2_ADDRCACHE_ENTRIES && !found; i++) {
        if (!c->e[i].used) {
            victim = i;
            found = true;                                    /* a free slot beats eviction */
        } else if (c->e[i].stamp < c->e[victim].stamp) {
            victim = i;                                      /* otherwise remember the LRU entry */
        }
    }
    c->e[victim].used = true;
    memcpy(c->e[victim].addr, addr, 6);
    c->e[victim].type = type;
    c->e[victim].stamp = ++c->clock;
}

bool bc2_addrcache_get(bc2_addrcache_t *c, const uint8_t addr[6], uint8_t *type)
{
    for (size_t i = 0; i < BC2_ADDRCACHE_ENTRIES; i++) {
        if (c->e[i].used && memcmp(c->e[i].addr, addr, 6) == 0) {
            c->e[i].stamp = ++c->clock;
            *type = c->e[i].type;
            return true;
        }
    }
    return false;
}
