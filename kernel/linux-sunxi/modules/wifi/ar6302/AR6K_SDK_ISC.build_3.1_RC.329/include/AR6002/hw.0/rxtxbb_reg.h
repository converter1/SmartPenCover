#ifndef _RXTXBB_REG_REG_H_
#define _RXTXBB_REG_REG_H_

#define RXTXBB_RXTXBB1_ADDRESS                   0x00000000
#define RXTXBB_RXTXBB1_OFFSET                    0x00000000
#define RXTXBB_RXTXBB1_SPARE_MSB                 31
#define RXTXBB_RXTXBB1_SPARE_LSB                 19
#define RXTXBB_RXTXBB1_SPARE_MASK                0xfff80000
#define RXTXBB_RXTXBB1_SPARE_GET(x)              (((x) & RXTXBB_RXTXBB1_SPARE_MASK) >> RXTXBB_RXTXBB1_SPARE_LSB)
#define RXTXBB_RXTXBB1_SPARE_SET(x)              (((x) << RXTXBB_RXTXBB1_SPARE_LSB) & RXTXBB_RXTXBB1_SPARE_MASK)
#define RXTXBB_RXTXBB1_FNOTCH_MSB                18
#define RXTXBB_RXTXBB1_FNOTCH_LSB                17
#define RXTXBB_RXTXBB1_FNOTCH_MASK               0x00060000
#define RXTXBB_RXTXBB1_FNOTCH_GET(x)             (((x) & RXTXBB_RXTXBB1_FNOTCH_MASK) >> RXTXBB_RXTXBB1_FNOTCH_LSB)
#define RXTXBB_RXTXBB1_FNOTCH_SET(x)             (((x) << RXTXBB_RXTXBB1_FNOTCH_LSB) & RXTXBB_RXTXBB1_FNOTCH_MASK)
#define RXTXBB_RXTXBB1_SEL_ATB_MSB               16
#define RXTXBB_RXTXBB1_SEL_ATB_LSB               9
#define RXTXBB_RXTXBB1_SEL_ATB_MASK              0x0001fe00
#define RXTXBB_RXTXBB1_SEL_ATB_GET(x)            (((x) & RXTXBB_RXTXBB1_SEL_ATB_MASK) >> RXTXBB_RXTXBB1_SEL_ATB_LSB)
#define RXTXBB_RXTXBB1_SEL_ATB_SET(x)            (((x) << RXTXBB_RXTXBB1_SEL_ATB_LSB) & RXTXBB_RXTXBB1_SEL_ATB_MASK)
#define RXTXBB_RXTXBB1_PDDACINTERFACE_MSB        8
#define RXTXBB_RXTXBB1_PDDACINTERFACE_LSB        8
#define RXTXBB_RXTXBB1_PDDACINTERFACE_MASK       0x00000100
#define RXTXBB_RXTXBB1_PDDACINTERFACE_GET(x)     (((x) & RXTXBB_RXTXBB1_PDDACINTERFACE_MASK) >> RXTXBB_RXTXBB1_PDDACINTERFACE_LSB)
#define RXTXBB_RXTXBB1_PDDACINTERFACE_SET(x)     (((x) << RXTXBB_RXTXBB1_PDDACINTERFACE_LSB) & RXTXBB_RXTXBB1_PDDACINTERFACE_MASK)
#define RXTXBB_RXTXBB1_PDV2I_MSB                 7
#define RXTXBB_RXTXBB1_PDV2I_LSB                 7
#define RXTXBB_RXTXBB1_PDV2I_MASK                0x00000080
#define RXTXBB_RXTXBB1_PDV2I_GET(x)              (((x) & RXTXBB_RXTXBB1_PDV2I_MASK) >> RXTXBB_RXTXBB1_PDV2I_LSB)
#define RXTXBB_RXTXBB1_PDV2I_SET(x)              (((x) << RXTXBB_RXTXBB1_PDV2I_LSB) & RXTXBB_RXTXBB1_PDV2I_MASK)
#define RXTXBB_RXTXBB1_PDI2V_MSB                 6
#define RXTXBB_RXTXBB1_PDI2V_LSB                 6
#define RXTXBB_RXTXBB1_PDI2V_MASK                0x00000040
#define RXTXBB_RXTXBB1_PDI2V_GET(x)              (((x) & RXTXBB_RXTXBB1_PDI2V_MASK) >> RXTXBB_RXTXBB1_PDI2V_LSB)
#define RXTXBB_RXTXBB1_PDI2V_SET(x)              (((x) << RXTXBB_RXTXBB1_PDI2V_LSB) & RXTXBB_RXTXBB1_PDI2V_MASK)
#define RXTXBB_RXTXBB1_PDRXTXBB_MSB              5
#define RXTXBB_RXTXBB1_PDRXTXBB_LSB              5
#define RXTXBB_RXTXBB1_PDRXTXBB_MASK             0x00000020
#define RXTXBB_RXTXBB1_PDRXTXBB_GET(x)           (((x) & RXTXBB_RXTXBB1_PDRXTXBB_MASK) >> RXTXBB_RXTXBB1_PDRXTXBB_LSB)
#define RXTXBB_RXTXBB1_PDRXTXBB_SET(x)           (((x) << RXTXBB_RXTXBB1_PDRXTXBB_LSB) & RXTXBB_RXTXBB1_PDRXTXBB_MASK)
#define RXTXBB_RXTXBB1_PDOFFSETLOQ_MSB           4
#define RXTXBB_RXTXBB1_PDOFFSETLOQ_LSB           4
#define RXTXBB_RXTXBB1_PDOFFSETLOQ_MASK          0x00000010
#define RXTXBB_RXTXBB1_PDOFFSETLOQ_GET(x)        (((x) & RXTXBB_RXTXBB1_PDOFFSETLOQ_MASK) >> RXTXBB_RXTXBB1_PDOFFSETLOQ_LSB)
#define RXTXBB_RXTXBB1_PDOFFSETLOQ_SET(x)        (((x) << RXTXBB_RXTXBB1_PDOFFSETLOQ_LSB) & RXTXBB_RXTXBB1_PDOFFSETLOQ_MASK)
#define RXTXBB_RXTXBB1_PDOFFSETHIQ_MSB           3
#define RXTXBB_RXTXBB1_PDOFFSETHIQ_LSB           3
#define RXTXBB_RXTXBB1_PDOFFSETHIQ_MASK          0x00000008
#define RXTXBB_RXTXBB1_PDOFFSETHIQ_GET(x)        (((x) & RXTXBB_RXTXBB1_PDOFFSETHIQ_MASK) >> RXTXBB_RXTXBB1_PDOFFSETHIQ_LSB)
#define RXTXBB_RXTXBB1_PDOFFSETHIQ_SET(x)        (((x) << RXTXBB_RXTXBB1_PDOFFSETHIQ_LSB) & RXTXBB_RXTXBB1_PDOFFSETHIQ_MASK)
#define RXTXBB_RXTXBB1_PDOFFSETI2V_MSB           2
#define RXTXBB_RXTXBB1_PDOFFSETI2V_LSB           2
#define RXTXBB_RXTXBB1_PDOFFSETI2V_MASK          0x00000004
#define RXTXBB_RXTXBB1_PDOFFSETI2V_GET(x)        (((x) & RXTXBB_RXTXBB1_PDOFFSETI2V_MASK) >> RXTXBB_RXTXBB1_PDOFFSETI2V_LSB)
#define RXTXBB_RXTXBB1_PDOFFSETI2V_SET(x)        (((x) << RXTXBB_RXTXBB1_PDOFFSETI2V_LSB) & RXTXBB_RXTXBB1_PDOFFSETI2V_MASK)
#define RXTXBB_RXTXBB1_PDLOQ_MSB                 1
#define RXTXBB_RXTXBB1_PDLOQ_LSB                 1
#define RXTXBB_RXTXBB1_PDLOQ_MASK                0x00000002
#define RXTXBB_RXTXBB1_PDLOQ_GET(x)              (((x) & RXTXBB_RXTXBB1_PDLOQ_MASK) >> RXTXBB_RXTXBB1_PDLOQ_LSB)
#define RXTXBB_RXTXBB1_PDLOQ_SET(x)              (((x) << RXTXBB_RXTXBB1_PDLOQ_LSB) & RXTXBB_RXTXBB1_PDLOQ_MASK)
#define RXTXBB_RXTXBB1_PDHIQ_MSB                 0
#define RXTXBB_RXTXBB1_PDHIQ_LSB                 0
#define RXTXBB_RXTXBB1_PDHIQ_MASK                0x00000001
#define RXTXBB_RXTXBB1_PDHIQ_GET(x)              (((x) & RXTXBB_RXTXBB1_PDHIQ_MASK) >> RXTXBB_RXTXBB1_PDHIQ_LSB)
#define RXTXBB_RXTXBB1_PDHIQ_SET(x)              (((x) << RXTXBB_RXTXBB1_PDHIQ_LSB) & RXTXBB_RXTXBB1_PDHIQ_MASK)

#define RXTXBB_RXTXBB2_ADDRESS                   0x00000004
#define RXTXBB_RXTXBB2_OFFSET                    0x00000004
#define RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_MSB    31
#define RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_LSB    29
#define RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_MASK   0xe0000000
#define RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_GET(x) (((x) & RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_MASK) >> RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_LSB)
#define RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_SET(x) (((x) << RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_LSB) & RXTXBB_RXTXBB2_IBN_37P5_OSHI_CTRL_MASK)
#define RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_MSB    28
#define RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_LSB    26
#define RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_MASK   0x1c000000
#define RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_GET(x) (((x) & RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_MASK) >> RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_LSB)
#define RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_SET(x) (((x) << RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_LSB) & RXTXBB_RXTXBB2_IBN_37P5_OSLO_CTRL_MASK)
#define RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_MSB   25
#define RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_LSB   23
#define RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_MASK  0x03800000
#define RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_GET(x) (((x) & RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_MASK) >> RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_LSB)
#define RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_SET(x) (((x) << RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_LSB) & RXTXBB_RXTXBB2_IBN_37P5_OSI2V_CTRL_MASK)
#define RXTXBB_RXTXBB2_SPARE_MSB                 22
#define RXTXBB_RXTXBB2_SPARE_LSB                 21
#define RXTXBB_RXTXBB2_SPARE_MASK                0x00600000
#define RXTXBB_RXTXBB2_SPARE_GET(x)              (((x) & RXTXBB_RXTXBB2_SPARE_MASK) >> RXTXBB_RXTXBB2_SPARE_LSB)
#define RXTXBB_RXTXBB2_SPARE_SET(x)              (((x) << RXTXBB_RXTXBB2_SPARE_LSB) & RXTXBB_RXTXBB2_SPARE_MASK)
#define RXTXBB_RXTXBB2_SHORTBUFFER_MSB           20
#define RXTXBB_RXTXBB2_SHORTBUFFER_LSB           20
#define RXTXBB_RXTXBB2_SHORTBUFFER_MASK          0x00100000
#define RXTXBB_RXTXBB2_SHORTBUFFER_GET(x)        (((x) & RXTXBB_RXTXBB2_SHORTBUFFER_MASK) >> RXTXBB_RXTXBB2_SHORTBUFFER_LSB)
#define RXTXBB_RXTXBB2_SHORTBUFFER_SET(x)        (((x) << RXTXBB_RXTXBB2_SHORTBUFFER_LSB) & RXTXBB_RXTXBB2_SHORTBUFFER_MASK)
#define RXTXBB_RXTXBB2_SELBUFFER_MSB             19
#define RXTXBB_RXTXBB2_SELBUFFER_LSB             19
#define RXTXBB_RXTXBB2_SELBUFFER_MASK            0x00080000
#define RXTXBB_RXTXBB2_SELBUFFER_GET(x)          (((x) & RXTXBB_RXTXBB2_SELBUFFER_MASK) >> RXTXBB_RXTXBB2_SELBUFFER_LSB)
#define RXTXBB_RXTXBB2_SELBUFFER_SET(x)          (((x) << RXTXBB_RXTXBB2_SELBUFFER_LSB) & RXTXBB_RXTXBB2_SELBUFFER_MASK)
#define RXTXBB_RXTXBB2_SEL_DAC_TEST_MSB          18
#define RXTXBB_RXTXBB2_SEL_DAC_TEST_LSB          18
#define RXTXBB_RXTXBB2_SEL_DAC_TEST_MASK         0x00040000
#define RXTXBB_RXTXBB2_SEL_DAC_TEST_GET(x)       (((x) & RXTXBB_RXTXBB2_SEL_DAC_TEST_MASK) >> RXTXBB_RXTXBB2_SEL_DAC_TEST_LSB)
#define RXTXBB_RXTXBB2_SEL_DAC_TEST_SET(x)       (((x) << RXTXBB_RXTXBB2_SEL_DAC_TEST_LSB) & RXTXBB_RXTXBB2_SEL_DAC_TEST_MASK)
#define RXTXBB_RXTXBB2_SEL_LOQ_TEST_MSB          17
#define RXTXBB_RXTXBB2_SEL_LOQ_TEST_LSB          17
#define RXTXBB_RXTXBB2_SEL_LOQ_TEST_MASK         0x00020000
#define RXTXBB_RXTXBB2_SEL_LOQ_TEST_GET(x)       (((x) & RXTXBB_RXTXBB2_SEL_LOQ_TEST_MASK) >> RXTXBB_RXTXBB2_SEL_LOQ_TEST_LSB)
#define RXTXBB_RXTXBB2_SEL_LOQ_TEST_SET(x)       (((x) << RXTXBB_RXTXBB2_SEL_LOQ_TEST_LSB) & RXTXBB_RXTXBB2_SEL_LOQ_TEST_MASK)
#define RXTXBB_RXTXBB2_SEL_HIQ_TEST_MSB          16
#define RXTXBB_RXTXBB2_SEL_HIQ_TEST_LSB          16
#define RXTXBB_RXTXBB2_SEL_HIQ_TEST_MASK         0x00010000
#define RXTXBB_RXTXBB2_SEL_HIQ_TEST_GET(x)       (((x) & RXTXBB_RXTXBB2_SEL_HIQ_TEST_MASK) >> RXTXBB_RXTXBB2_SEL_HIQ_TEST_LSB)
#define RXTXBB_RXTXBB2_SEL_HIQ_TEST_SET(x)       (((x) << RXTXBB_RXTXBB2_SEL_HIQ_TEST_LSB) & RXTXBB_RXTXBB2_SEL_HIQ_TEST_MASK)
#define RXTXBB_RXTXBB2_SEL_I2V_TEST_MSB          15
#define RXTXBB_RXTXBB2_SEL_I2V_TEST_LSB          15
#define RXTXBB_RXTXBB2_SEL_I2V_TEST_MASK         0x00008000
#define RXTXBB_RXTXBB2_SEL_I2V_TEST_GET(x)       (((x) & RXTXBB_RXTXBB2_SEL_I2V_TEST_MASK) >> RXTXBB_RXTXBB2_SEL_I2V_TEST_LSB)
#define RXTXBB_RXTXBB2_SEL_I2V_TEST_SET(x)       (((x) << RXTXBB_RXTXBB2_SEL_I2V_TEST_LSB) & RXTXBB_RXTXBB2_SEL_I2V_TEST_MASK)
#define RXTXBB_RXTXBB2_CMSEL_MSB                 14
#define RXTXBB_RXTXBB2_CMSEL_LSB                 13
#define RXTXBB_RXTXBB2_CMSEL_MASK                0x00006000
#define RXTXBB_RXTXBB2_CMSEL_GET(x)              (((x) & RXTXBB_RXTXBB2_CMSEL_MASK) >> RXTXBB_RXTXBB2_CMSEL_LSB)
#define RXTXBB_RXTXBB2_CMSEL_SET(x)              (((x) << RXTXBB_RXTXBB2_CMSEL_LSB) & RXTXBB_RXTXBB2_CMSEL_MASK)
#define RXTXBB_RXTXBB2_FILTERFC_MSB              12
#define RXTXBB_RXTXBB2_FILTERFC_LSB              8
#define RXTXBB_RXTXBB2_FILTERFC_MASK             0x00001f00
#define RXTXBB_RXTXBB2_FILTERFC_GET(x)           (((x) & RXTXBB_RXTXBB2_FILTERFC_MASK) >> RXTXBB_RXTXBB2_FILTERFC_LSB)
#define RXTXBB_RXTXBB2_FILTERFC_SET(x)           (((x) << RXTXBB_RXTXBB2_FILTERFC_LSB) & RXTXBB_RXTXBB2_FILTERFC_MASK)
#define RXTXBB_RXTXBB2_LOCALFILTERTUNING_MSB     7
#define RXTXBB_RXTXBB2_LOCALFILTERTUNING_LSB     7
#define RXTXBB_RXTXBB2_LOCALFILTERTUNING_MASK    0x00000080
#define RXTXBB_RXTXBB2_LOCALFILTERTUNING_GET(x)  (((x) & RXTXBB_RXTXBB2_LOCALFILTERTUNING_MASK) >> RXTXBB_RXTXBB2_LOCALFILTERTUNING_LSB)
#define RXTXBB_RXTXBB2_LOCALFILTERTUNING_SET(x)  (((x) << RXTXBB_RXTXBB2_LOCALFILTERTUNING_LSB) & RXTXBB_RXTXBB2_LOCALFILTERTUNING_MASK)
#define RXTXBB_RXTXBB2_FILTERDOUBLEBW_MSB        6
#define RXTXBB_RXTXBB2_FILTERDOUBLEBW_LSB        6
#define RXTXBB_RXTXBB2_FILTERDOUBLEBW_MASK       0x00000040
#define RXTXBB_RXTXBB2_FILTERDOUBLEBW_GET(x)     (((x) & RXTXBB_RXTXBB2_FILTERDOUBLEBW_MASK) >> RXTXBB_RXTXBB2_FILTERDOUBLEBW_LSB)
#define RXTXBB_RXTXBB2_FILTERDOUBLEBW_SET(x)     (((x) << RXTXBB_RXTXBB2_FILTERDOUBLEBW_LSB) & RXTXBB_RXTXBB2_FILTERDOUBLEBW_MASK)
#define RXTXBB_RXTXBB2_PATH2HIQ_EN_MSB           5
#define RXTXBB_RXTXBB2_PATH2HIQ_EN_LSB           5
#define RXTXBB_RXTXBB2_PATH2HIQ_EN_MASK          0x00000020
#define RXTXBB_RXTXBB2_PATH2HIQ_EN_GET(x)        (((x) & RXTXBB_RXTXBB2_PATH2HIQ_EN_MASK) >> RXTXBB_RXTXBB2_PATH2HIQ_EN_LSB)
#define RXTXBB_RXTXBB2_PATH2HIQ_EN_SET(x)        (((x) << RXTXBB_RXTXBB2_PATH2HIQ_EN_LSB) & RXTXBB_RXTXBB2_PATH2HIQ_EN_MASK)
#define RXTXBB_RXTXBB2_PATH1HIQ_EN_MSB           4
#define RXTXBB_RXTXBB2_PATH1HIQ_EN_LSB           4
#define RXTXBB_RXTXBB2_PATH1HIQ_EN_MASK          0x00000010
#define RXTXBB_RXTXBB2_PATH1HIQ_EN_GET(x)        (((x) & RXTXBB_RXTXBB2_PATH1HIQ_EN_MASK) >> RXTXBB_RXTXBB2_PATH1HIQ_EN_LSB)
#define RXTXBB_RXTXBB2_PATH1HIQ_EN_SET(x)        (((x) << RXTXBB_RXTXBB2_PATH1HIQ_EN_LSB) & RXTXBB_RXTXBB2_PATH1HIQ_EN_MASK)
#define RXTXBB_RXTXBB2_PATH3LOQ_EN_MSB           3
#define RXTXBB_RXTXBB2_PATH3LOQ_EN_LSB           3
#define RXTXBB_RXTXBB2_PATH3LOQ_EN_MASK          0x00000008
#define RXTXBB_RXTXBB2_PATH3LOQ_EN_GET(x)        (((x) & RXTXBB_RXTXBB2_PATH3LOQ_EN_MASK) >> RXTXBB_RXTXBB2_PATH3LOQ_EN_LSB)
#define RXTXBB_RXTXBB2_PATH3LOQ_EN_SET(x)        (((x) << RXTXBB_RXTXBB2_PATH3LOQ_EN_LSB) & RXTXBB_RXTXBB2_PATH3LOQ_EN_MASK)
#define RXTXBB_RXTXBB2_PATH2LOQ_EN_MSB           2
#define RXTXBB_RXTXBB2_PATH2LOQ_EN_LSB           2
#define RXTXBB_RXTXBB2_PATH2LOQ_EN_MASK          0x00000004
#define RXTXBB_RXTXBB2_PATH2LOQ_EN_GET(x)        (((x) & RXTXBB_RXTXBB2_PATH2LOQ_EN_MASK) >> RXTXBB_RXTXBB2_PATH2LOQ_EN_LSB)
#define RXTXBB_RXTXBB2_PATH2LOQ_EN_SET(x)        (((x) << RXTXBB_RXTXBB2_PATH2LOQ_EN_LSB) & RXTXBB_RXTXBB2_PATH2LOQ_EN_MASK)
#define RXTXBB_RXTXBB2_PATH1LOQ_EN_MSB           1
#define RXTXBB_RXTXBB2_PATH1LOQ_EN_LSB           1
#define RXTXBB_RXTXBB2_PATH1LOQ_EN_MASK          0x00000002
#define RXTXBB_RXTXBB2_PATH1LOQ_EN_GET(x)        (((x) & RXTXBB_RXTXBB2_PATH1LOQ_EN_MASK) >> RXTXBB_RXTXBB2_PATH1LOQ_EN_LSB)
#define RXTXBB_RXTXBB2_PATH1LOQ_EN_SET(x)        (((x) << RXTXBB_RXTXBB2_PATH1LOQ_EN_LSB) & RXTXBB_RXTXBB2_PATH1LOQ_EN_MASK)
#define RXTXBB_RXTXBB2_PATH_OVERRIDE_MSB         0
#define RXTXBB_RXTXBB2_PATH_OVERRIDE_LSB         0
#define RXTXBB_RXTXBB2_PATH_OVERRIDE_MASK        0x00000001
#define RXTXBB_RXTXBB2_PATH_OVERRIDE_GET(x)      (((x) & RXTXBB_RXTXBB2_PATH_OVERRIDE_MASK) >> RXTXBB_RXTXBB2_PATH_OVERRIDE_LSB)
#define RXTXBB_RXTXBB2_PATH_OVERRIDE_SET(x)      (((x) << RXTXBB_RXTXBB2_PATH_OVERRIDE_LSB) & RXTXBB_RXTXBB2_PATH_OVERRIDE_MASK)

#define RXTXBB_RXTXBB3_ADDRESS                   0x00000008
#define RXTXBB_RXTXBB3_OFFSET                    0x00000008
#define RXTXBB_RXTXBB3_SPARE_MSB                 31
#define RXTXBB_RXTXBB3_SPARE_LSB                 27
#define RXTXBB_RXTXBB3_SPARE_MASK                0xf8000000
#define RXTXBB_RXTXBB3_SPARE_GET(x)              (((x) & RXTXBB_RXTXBB3_SPARE_MASK) >> RXTXBB_RXTXBB3_SPARE_LSB)
#define RXTXBB_RXTXBB3_SPARE_SET(x)              (((x) << RXTXBB_RXTXBB3_SPARE_LSB) & RXTXBB_RXTXBB3_SPARE_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_MSB 26
#define RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_LSB 24
#define RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_MASK 0x07000000
#define RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_GET(x) (((x) & RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_SET(x) (((x) << RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_CM_BUFAMP_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_MSB    23
#define RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_LSB    21
#define RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_MASK   0x00e00000
#define RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_GET(x) (((x) & RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_SET(x) (((x) << RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_BKV2I_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_MSB      20
#define RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_LSB      18
#define RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_MASK     0x001c0000
#define RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_GET(x)   (((x) & RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_SET(x)   (((x) << RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_I2V_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_MSB      17
#define RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_LSB      15
#define RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_MASK     0x00038000
#define RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_GET(x)   (((x) & RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_SET(x)   (((x) << RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_HI1_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_MSB      14
#define RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_LSB      12
#define RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_MASK     0x00007000
#define RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_GET(x)   (((x) & RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_SET(x)   (((x) << RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_HI2_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_MSB      11
#define RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_LSB      9
#define RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_MASK     0x00000e00
#define RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_GET(x)   (((x) & RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_SET(x)   (((x) << RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_LO1_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_MSB      8
#define RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_LSB      6
#define RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_MASK     0x000001c0
#define RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_GET(x)   (((x) & RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_SET(x)   (((x) << RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_25U_LO2_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_MSB     5
#define RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_LSB     3
#define RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_MASK    0x00000038
#define RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_GET(x)  (((x) & RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_MASK) >> RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_SET(x)  (((x) << RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_LSB) & RXTXBB_RXTXBB3_IBRN_12P5_CM_CTRL_MASK)
#define RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_MSB    2
#define RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_LSB    0
#define RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_MASK   0x00000007
#define RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_GET(x) (((x) & RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_MASK) >> RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_LSB)
#define RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_SET(x) (((x) << RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_LSB) & RXTXBB_RXTXBB3_IBN_100U_TEST_CTRL_MASK)

#define RXTXBB_RXTXBB4_ADDRESS                   0x0000000c
#define RXTXBB_RXTXBB4_OFFSET                    0x0000000c
#define RXTXBB_RXTXBB4_SPARE_MSB                 31
#define RXTXBB_RXTXBB4_SPARE_LSB                 31
#define RXTXBB_RXTXBB4_SPARE_MASK                0x80000000
#define RXTXBB_RXTXBB4_SPARE_GET(x)              (((x) & RXTXBB_RXTXBB4_SPARE_MASK) >> RXTXBB_RXTXBB4_SPARE_LSB)
#define RXTXBB_RXTXBB4_SPARE_SET(x)              (((x) << RXTXBB_RXTXBB4_SPARE_LSB) & RXTXBB_RXTXBB4_SPARE_MASK)
#define RXTXBB_RXTXBB4_LOCALOFFSET_MSB           30
#define RXTXBB_RXTXBB4_LOCALOFFSET_LSB           30
#define RXTXBB_RXTXBB4_LOCALOFFSET_MASK          0x40000000
#define RXTXBB_RXTXBB4_LOCALOFFSET_GET(x)        (((x) & RXTXBB_RXTXBB4_LOCALOFFSET_MASK) >> RXTXBB_RXTXBB4_LOCALOFFSET_LSB)
#define RXTXBB_RXTXBB4_LOCALOFFSET_SET(x)        (((x) << RXTXBB_RXTXBB4_LOCALOFFSET_LSB) & RXTXBB_RXTXBB4_LOCALOFFSET_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRHII_MSB           29
#define RXTXBB_RXTXBB4_OFSTCORRHII_LSB           25
#define RXTXBB_RXTXBB4_OFSTCORRHII_MASK          0x3e000000
#define RXTXBB_RXTXBB4_OFSTCORRHII_GET(x)        (((x) & RXTXBB_RXTXBB4_OFSTCORRHII_MASK) >> RXTXBB_RXTXBB4_OFSTCORRHII_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRHII_SET(x)        (((x) << RXTXBB_RXTXBB4_OFSTCORRHII_LSB) & RXTXBB_RXTXBB4_OFSTCORRHII_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRHIQ_MSB           24
#define RXTXBB_RXTXBB4_OFSTCORRHIQ_LSB           20
#define RXTXBB_RXTXBB4_OFSTCORRHIQ_MASK          0x01f00000
#define RXTXBB_RXTXBB4_OFSTCORRHIQ_GET(x)        (((x) & RXTXBB_RXTXBB4_OFSTCORRHIQ_MASK) >> RXTXBB_RXTXBB4_OFSTCORRHIQ_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRHIQ_SET(x)        (((x) << RXTXBB_RXTXBB4_OFSTCORRHIQ_LSB) & RXTXBB_RXTXBB4_OFSTCORRHIQ_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRLOI_MSB           19
#define RXTXBB_RXTXBB4_OFSTCORRLOI_LSB           15
#define RXTXBB_RXTXBB4_OFSTCORRLOI_MASK          0x000f8000
#define RXTXBB_RXTXBB4_OFSTCORRLOI_GET(x)        (((x) & RXTXBB_RXTXBB4_OFSTCORRLOI_MASK) >> RXTXBB_RXTXBB4_OFSTCORRLOI_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRLOI_SET(x)        (((x) << RXTXBB_RXTXBB4_OFSTCORRLOI_LSB) & RXTXBB_RXTXBB4_OFSTCORRLOI_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRLOQ_MSB           14
#define RXTXBB_RXTXBB4_OFSTCORRLOQ_LSB           10
#define RXTXBB_RXTXBB4_OFSTCORRLOQ_MASK          0x00007c00
#define RXTXBB_RXTXBB4_OFSTCORRLOQ_GET(x)        (((x) & RXTXBB_RXTXBB4_OFSTCORRLOQ_MASK) >> RXTXBB_RXTXBB4_OFSTCORRLOQ_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRLOQ_SET(x)        (((x) << RXTXBB_RXTXBB4_OFSTCORRLOQ_LSB) & RXTXBB_RXTXBB4_OFSTCORRLOQ_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRI2VI_MSB          9
#define RXTXBB_RXTXBB4_OFSTCORRI2VI_LSB          5
#define RXTXBB_RXTXBB4_OFSTCORRI2VI_MASK         0x000003e0
#define RXTXBB_RXTXBB4_OFSTCORRI2VI_GET(x)       (((x) & RXTXBB_RXTXBB4_OFSTCORRI2VI_MASK) >> RXTXBB_RXTXBB4_OFSTCORRI2VI_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRI2VI_SET(x)       (((x) << RXTXBB_RXTXBB4_OFSTCORRI2VI_LSB) & RXTXBB_RXTXBB4_OFSTCORRI2VI_MASK)
#define RXTXBB_RXTXBB4_OFSTCORRI2VQ_MSB          4
#define RXTXBB_RXTXBB4_OFSTCORRI2VQ_LSB          0
#define RXTXBB_RXTXBB4_OFSTCORRI2VQ_MASK         0x0000001f
#define RXTXBB_RXTXBB4_OFSTCORRI2VQ_GET(x)       (((x) & RXTXBB_RXTXBB4_OFSTCORRI2VQ_MASK) >> RXTXBB_RXTXBB4_OFSTCORRI2VQ_LSB)
#define RXTXBB_RXTXBB4_OFSTCORRI2VQ_SET(x)       (((x) << RXTXBB_RXTXBB4_OFSTCORRI2VQ_LSB) & RXTXBB_RXTXBB4_OFSTCORRI2VQ_MASK)


#ifndef __ASSEMBLER__

typedef struct rxtxbb_reg_reg_s {
  volatile unsigned int rxtxbb_rxtxbb1;
  volatile unsigned int rxtxbb_rxtxbb2;
  volatile unsigned int rxtxbb_rxtxbb3;
  volatile unsigned int rxtxbb_rxtxbb4;
} rxtxbb_reg_reg_t;

#endif /* __ASSEMBLER__ */

#endif /* _RXTXBB_REG_H_ */
