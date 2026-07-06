/*
 * CI-only stub of the IgH (EtherLab) EtherCAT master userspace API.
 *
 * The real `ecrt.h` + libethercat live only on the bench Pi (/opt/etherlab),
 * so the hw endpoint (`--features hw`) is built nowhere else. `scripts/ci.sh
 * rust-ethercat-hw` points IGH_DIR at this directory and runs `cargo check`,
 * which compiles `csrc/libecrt_igh.c` against these declarations (catching C
 * compile + Rust binary errors) without linking libethercat.
 *
 * It must MIRROR the subset of the real ecrt.h that `libecrt_igh.c` uses — the
 * signatures here are what type-check the backend's calls, so keep them
 * faithful to IgH. If the backend starts using a new ecrt_* symbol, add it
 * here (the C compile fails loudly until you do). This is a compile/typecheck
 * gate, not a behavioural or ABI-fidelity check.
 */
#ifndef ECRT_CI_STUB_H
#define ECRT_CI_STUB_H

#include <stdint.h>
#include <stddef.h>

#define EC_END ~0U

typedef struct ec_master ec_master_t;
typedef struct ec_domain ec_domain_t;
typedef struct ec_slave_config ec_slave_config_t;
typedef struct ec_reg_request ec_reg_request_t;

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
} ec_pdo_entry_info_t;

typedef struct {
    uint16_t index;
    unsigned int n_entries;
    ec_pdo_entry_info_t *entries;
} ec_pdo_info_t;

typedef enum {
    EC_DIR_INVALID,
    EC_DIR_OUTPUT,
    EC_DIR_INPUT,
    EC_DIR_COUNT,
} ec_direction_t;

typedef enum {
    EC_WD_DEFAULT,
    EC_WD_ENABLE,
    EC_WD_DISABLE,
} ec_watchdog_mode_t;

typedef struct {
    uint8_t index;
    ec_direction_t dir;
    unsigned int n_pdos;
    ec_pdo_info_t *pdos;
    ec_watchdog_mode_t watchdog_mode;
} ec_sync_info_t;

typedef struct {
    unsigned int working_counter;
    unsigned int redundancy_active;
} ec_domain_state_t;

typedef struct {
    unsigned int al_state;
    unsigned int online;
    unsigned int operational;
} ec_slave_config_state_t;

typedef enum {
    EC_REQUEST_UNUSED,
    EC_REQUEST_BUSY,
    EC_REQUEST_SUCCESS,
    EC_REQUEST_ERROR,
} ec_request_state_t;

#define EC_READ_U16(DATA) (*(const uint16_t *) (DATA))
#define EC_READ_S16(DATA) (*(const int16_t *) (DATA))
#define EC_READ_S32(DATA) (*(const int32_t *) (DATA))
#define EC_WRITE_U16(DATA, VAL) do { *(uint16_t *) (DATA) = (uint16_t) (VAL); } while (0)
#define EC_WRITE_U32(DATA, VAL) do { *(uint32_t *) (DATA) = (uint32_t) (VAL); } while (0)
#define EC_WRITE_S16(DATA, VAL) do { *(int16_t *) (DATA) = (int16_t) (VAL); } while (0)
#define EC_WRITE_S32(DATA, VAL) do { *(int32_t *) (DATA) = (int32_t) (VAL); } while (0)

ec_master_t *ecrt_request_master(unsigned int master_index);
void ecrt_release_master(ec_master_t *master);

ec_domain_t *ecrt_master_create_domain(ec_master_t *master);
ec_slave_config_t *ecrt_master_slave_config(ec_master_t *master, uint16_t alias,
        uint16_t position, uint32_t vendor_id, uint32_t product_code);
int ecrt_master_activate(ec_master_t *master);
void ecrt_master_deactivate(ec_master_t *master);
void ecrt_master_application_time(ec_master_t *master, uint64_t app_time);
void ecrt_master_receive(ec_master_t *master);
void ecrt_master_send(ec_master_t *master);
void ecrt_master_sync_reference_clock(ec_master_t *master);
void ecrt_master_sync_slave_clocks(ec_master_t *master);
int ecrt_master_sdo_upload(ec_master_t *master, uint16_t position, uint16_t index,
        uint8_t subindex, uint8_t *target, size_t target_size, size_t *result_size,
        uint32_t *abort_code);
int ecrt_master_sdo_download(ec_master_t *master, uint16_t position, uint16_t index,
        uint8_t subindex, const uint8_t *data, size_t data_size, uint32_t *abort_code);

uint8_t *ecrt_domain_data(ec_domain_t *domain);
size_t ecrt_domain_size(ec_domain_t *domain);
void ecrt_domain_process(ec_domain_t *domain);
void ecrt_domain_queue(ec_domain_t *domain);
void ecrt_domain_state(const ec_domain_t *domain, ec_domain_state_t *state);

int ecrt_slave_config_pdos(ec_slave_config_t *sc, unsigned int n_syncs,
        const ec_sync_info_t syncs[]);
int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t *sc, uint16_t entry_index,
        uint8_t entry_subindex, ec_domain_t *domain, unsigned int *bit_position);
int ecrt_slave_config_sdo8(ec_slave_config_t *sc, uint16_t index, uint8_t subindex,
        uint8_t value);
int ecrt_slave_config_sdo16(ec_slave_config_t *sc, uint16_t index, uint8_t subindex,
        uint16_t value);
int ecrt_slave_config_dc(ec_slave_config_t *sc, uint16_t assign_activate,
        uint32_t sync0_cycle, int32_t sync0_shift, uint32_t sync1_cycle,
        int32_t sync1_shift);
ec_reg_request_t *ecrt_slave_config_create_reg_request(ec_slave_config_t *sc, size_t size);
int ecrt_slave_config_state(const ec_slave_config_t *sc, ec_slave_config_state_t *state);

void ecrt_reg_request_read(ec_reg_request_t *req, uint16_t address, size_t size);
ec_request_state_t ecrt_reg_request_state(const ec_reg_request_t *req);
uint8_t *ecrt_reg_request_data(const ec_reg_request_t *req);

#endif
