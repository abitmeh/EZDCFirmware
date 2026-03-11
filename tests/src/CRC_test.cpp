#include "Utilities/CRC.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

// ─────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────

// Reference test sequence — a DS18B20-style 7-byte payload
static constexpr uint8_t kTestData[] = {0x28, 0xFF, 0x5E, 0x75, 0x16, 0x16, 0x04};
static constexpr size_t kTestDataLen = sizeof(kTestData);

// ─────────────────────────────────────────────
// Per-variant parameterised fixture
// ─────────────────────────────────────────────

struct CRCVariantParams {
    CRCType type;
    const char* name;
    uint8_t table_01;   // crc8({0x01})
    uint8_t table_ff;   // crc8({0xFF})
    uint8_t multi_crc;  // crc8(kTestData)
};

class CRC8VariantTest : public ::testing::TestWithParam<CRCVariantParams> {};

// Convenience: compute crc8 for a given variant via the table directly
static uint8_t compute(CRCType type, const uint8_t* data, size_t len, uint8_t prev = 0) {
    switch (type) {
        case CRCType::DallasMaxim:
            return crc8<CRCType::DallasMaxim>(data, len, prev);
        case CRCType::Autosar:
            return crc8<CRCType::Autosar>(data, len, prev);
        case CRCType::Bluetooth:
            return crc8<CRCType::Bluetooth>(data, len, prev);
        case CRCType::CCITT:
            return crc8<CRCType::CCITT>(data, len, prev);
        case CRCType::DARC:
            return crc8<CRCType::DARC>(data, len, prev);
        case CRCType::GSM_B:
            return crc8<CRCType::GSM_B>(data, len, prev);
        case CRCType::J1850:
            return crc8<CRCType::J1850>(data, len, prev);
        case CRCType::WCDMA:
            return crc8<CRCType::WCDMA>(data, len, prev);
    }
    return 0;
}

static const uint8_t* get_table(CRCType type) {
    switch (type) {
        case CRCType::DallasMaxim:
            return table_lookup<CRCType::DallasMaxim>::table;
        case CRCType::Autosar:
            return table_lookup<CRCType::Autosar>::table;
        case CRCType::Bluetooth:
            return table_lookup<CRCType::Bluetooth>::table;
        case CRCType::CCITT:
            return table_lookup<CRCType::CCITT>::table;
        case CRCType::DARC:
            return table_lookup<CRCType::DARC>::table;
        case CRCType::GSM_B:
            return table_lookup<CRCType::GSM_B>::table;
        case CRCType::J1850:
            return table_lookup<CRCType::J1850>::table;
        case CRCType::WCDMA:
            return table_lookup<CRCType::WCDMA>::table;
    }
    return nullptr;
}

// ─────────────────────────────────────────────
// Parameterised tests — run for every variant
// ─────────────────────────────────────────────

TEST_P(CRC8VariantTest, EmptyInput) {
    EXPECT_EQ(compute(GetParam().type, nullptr, 0), 0x00);
}

TEST_P(CRC8VariantTest, SingleZeroByte) {
    const uint8_t data[] = {0x00};
    EXPECT_EQ(compute(GetParam().type, data, 1), 0x00);
}

TEST_P(CRC8VariantTest, SingleOneByte) {
    const uint8_t data[] = {0x01};
    EXPECT_EQ(compute(GetParam().type, data, 1), GetParam().table_01);
}

TEST_P(CRC8VariantTest, SingleFFByte) {
    const uint8_t data[] = {0xFF};
    EXPECT_EQ(compute(GetParam().type, data, 1), GetParam().table_ff);
}

TEST_P(CRC8VariantTest, KnownSequence) {
    EXPECT_EQ(compute(GetParam().type, kTestData, kTestDataLen), GetParam().multi_crc);
}

TEST_P(CRC8VariantTest, AppendCRCGivesZero) {
    // Appending the CRC to the message and recomputing should yield 0x00
    uint8_t buf[kTestDataLen + 1];
    memcpy(buf, kTestData, kTestDataLen);
    buf[kTestDataLen] = GetParam().multi_crc;
    EXPECT_EQ(compute(GetParam().type, buf, kTestDataLen + 1), 0x00);
}

TEST_P(CRC8VariantTest, SingleByteMatchesTable) {
    const uint8_t* table = get_table(GetParam().type);
    for (int i = 0; i < 256; i++) {
        const uint8_t byte = static_cast<uint8_t>(i);
        EXPECT_EQ(compute(GetParam().type, &byte, 1), table[i]) << "Failed for byte 0x" << std::hex << i << " in " << GetParam().name;
    }
}

TEST_P(CRC8VariantTest, TableFirstEntryIsZero) {
    EXPECT_EQ(get_table(GetParam().type)[0], 0x00);
}

TEST_P(CRC8VariantTest, TableValuesAreUnique) {
    // A valid CRC table is a permutation of 0–255
    const uint8_t* table = get_table(GetParam().type);
    std::array<int, 256> counts{};
    for (int i = 0; i < 256; i++)
        counts[table[i]]++;
    for (int i = 0; i < 256; i++) {
        EXPECT_EQ(counts[i], 1) << "Value " << i << " appears " << counts[i] << " times in " << GetParam().name << " table";
    }
}

TEST_P(CRC8VariantTest, Deterministic) {
    const uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    EXPECT_EQ(compute(GetParam().type, data, 4), compute(GetParam().type, data, 4));
}

TEST_P(CRC8VariantTest, OrderMatters) {
    const uint8_t data1[] = {0x01, 0x02};
    const uint8_t data2[] = {0x02, 0x01};
    EXPECT_NE(compute(GetParam().type, data1, 2), compute(GetParam().type, data2, 2));
}

TEST_P(CRC8VariantTest, SingleByteChangeCausesDifferentCRC) {
    const uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
    const uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};
    EXPECT_NE(compute(GetParam().type, data1, 4), compute(GetParam().type, data2, 4));
}

TEST_P(CRC8VariantTest, ContinuationMatchesFullCompute) {
    // crc8(a+b) == crc8(b, previousCRC=crc8(a))
    const uint8_t part1[] = {0x01, 0x02, 0x03};
    const uint8_t part2[] = {0x04, 0x05, 0x06};
    uint8_t combined[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    const uint8_t crc_part1 = compute(GetParam().type, part1, 3);
    const uint8_t crc_chained = compute(GetParam().type, part2, 3, crc_part1);
    const uint8_t crc_full = compute(GetParam().type, combined, 6);

    EXPECT_EQ(crc_chained, crc_full) << "Chained CRC doesn't match full CRC for " << GetParam().name;
}

INSTANTIATE_TEST_SUITE_P(
    AllVariants, CRC8VariantTest,
    ::testing::Values(CRCVariantParams{CRCType::DallasMaxim, "DallasMaxim", 0x5E, 0x35, 0xEA}, CRCVariantParams{CRCType::Autosar, "Autosar", 0xC7, 0x42, 0xDA},
                      CRCVariantParams{CRCType::Bluetooth, "Bluetooth", 0x6B, 0x9F, 0xBF}, CRCVariantParams{CRCType::CCITT, "CCITT", 0x91, 0xCF, 0x1A},
                      CRCVariantParams{CRCType::DARC, "DARC", 0x72, 0xC6, 0x19}, CRCVariantParams{CRCType::GSM_B, "GSM_B", 0x7A, 0xCA, 0xB9},
                      CRCVariantParams{CRCType::J1850, "J1850", 0x64, 0x23, 0x7A}, CRCVariantParams{CRCType::WCDMA, "WCDMA", 0xD0, 0xDE, 0xE6}),
    [](const ::testing::TestParamInfo<CRCVariantParams>& info) { return info.param.name; });
