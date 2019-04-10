/********************************************
 ** Copyright (C) 2018 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: ili9881_hd_dsi_vdo_txd_csot_zal1810.c
 ** Description: Source file for LCD driver
 **          To Control LCD driver
 ** Version :1.0
 ** Date : 2018/10/03
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2018/10/03   Liyan@ODM_HQ   Source file for LCD driver
 ********************************************/

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif
extern int g_gesture;
#define LCM_ID (0x98)

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)        (lcm_util.mdelay(n))
#define UDELAY(n)        (lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
      lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
#define SET_LCD_BIAS_EN(en, seq, value)                           lcm_util.set_lcd_bias_en(en, seq, value)
#endif

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <soc/oppo/device_info.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH        (720)
#define FRAME_HEIGHT    (1520)
#define LCM_DENSITY        (320)

#define LCM_PHYSICAL_WIDTH        (67608)
#define LCM_PHYSICAL_HEIGHT        (142728)

#define REGFLAG_DELAY        0xFFFC
#define REGFLAG_UDELAY    0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW    0xFFFE
#define REGFLAG_RESET_HIGH    0xFFFF

extern void lcd_resume_load_ili_fw(void);
extern int ilitek_tp;
extern unsigned int esd_recovery_backlight_level;
static int cabc_lastlevel = 1;
extern int pmi_lcd_bias_vsp_is_enabled(void);
extern int pmi_lcd_bias_vsn_is_enabled(void);

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define EXPONENTIAL_REMAPPING 1

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static int backlight_ili9881h_buf[] = {
    0, 18, 18, 111, 212, 313, 315, 317, 319, 321, 323, 325, 327, 329, 331, 333, 335, 337, 339, 341, \
    343, 345, 347, 349, 351, 353, 355, 357, 359, 361, 363, 365, 367, 369, 371, 373, 375, 377, 379, 381, \
    383, 385, 387, 389, 391, 393, 395, 397, 399, 401, 403, 405, 407, 409, 411, 413, 415, 417, 419, 421, \
    423, 425, 427, 429, 431, 433, 436, 438, 440, 442, 444, 446, 448, 450, 452, 454, 456, 458, 460, 462, \
    464, 466, 468, 470, 472, 474, 476, 478, 480, 482, 484, 486, 488, 490, 492, 494, 496, 498, 500, 502, \
    504, 506, 508, 510, 512, 514, 516, 518, 520, 522, 524, 526, 528, 530, 532, 534, 536, 538, 540, 542, \
    544, 546, 548, 550, 552, 554, 556, 558, 560, 562, 564, 566, 568, 570, 572, 574, 576, 578, 580, 582, \
    584, 586, 588, 590, 592, 594, 596, 598, 600, 602, 604, 606, 608, 610, 612, 614, 616, 618, 620, 622, \
    624, 626, 628, 630, 632, 634, 636, 638, 640, 642, 644, 646, 648, 650, 652, 654, 656, 658, 660, 662, \
    664, 666, 668, 670, 672, 674, 677, 679, 681, 683, 685, 687, 689, 691, 693, 695, 697, 699, 701, 703, \
    705, 707, 709, 711, 713, 715, 717, 719, 721, 723, 725, 727, 729, 731, 733, 735, 737, 739, 741, 743, \
    745, 747, 749, 751, 753, 755, 757, 759, 761, 763, 765, 767, 769, 771, 773, 775, 777, 779, 781, 783, \
    785, 787, 789, 791, 793, 795, 797, 798, 799, 800, 801, 803, 804, 805, 806, 807, 808, 809, 810, 812, \
    813, 814, 815, 816, 817, 818, 819, 821, 822, 823, 824, 825, 826, 827, 828, 829, 831, 832, 833, 834, \
    835, 836, 837, 838, 840, 841, 842, 843, 844, 845, 846, 847, 848, 850, 851, 852, 853, 854, 855, 856, \
    857, 859, 860, 861, 862, 863, 864, 865, 866, 868, 869, 870, 871, 872, 873, 874, 875, 876, 878, 879, \
    880, 881, 882, 883, 884, 885, 887, 888, 889, 890, 891, 892, 893, 894, 895, 897, 898, 899, 900, 901, \
    902, 903, 904, 906, 907, 908, 909, 910, 911, 912, 913, 915, 916, 917, 918, 919, 920, 921, 922, 923, \
    925, 926, 927, 928, 929, 930, 931, 932, 934, 935, 936, 937, 938, 939, 940, 941, 942, 944, 945, 946, \
    947, 948, 949, 950, 951, 953, 954, 955, 956, 957, 958, 959, 960, 962, 963, 964, 965, 966, 967, 968, \
    969, 970, 970, 971, 972, 973, 974, 975, 976, 977, 977, 978, 979, 980, 981, 982, 983, 984, 984, 985, \
    986, 987, 988, 989, 990, 991, 992, 992, 993, 994, 995, 996, 997, 998, 999, 999, 1000, 1001, 1002, 1003, \
    1004, 1005, 1006, 1007, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1014, 1015, 1016, 1017, 1018, 1019, 1020, 1021, \
    1021, 1022, 1023, 1024, 1025, 1026, 1027, 1028, 1029, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1036, 1037, 1038, \
    1039, 1040, 1041, 1042, 1043, 1043, 1044, 1045, 1046, 1047, 1048, 1049, 1050, 1051, 1051, 1052, 1053, 1054, 1055, 1056, \
    1057, 1058, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066, 1066, 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1073, \
    1074, 1075, 1076, 1077, 1078, 1079, 1080, 1080, 1081, 1082, 1083, 1084, 1085, 1086, 1087, 1088, 1088, 1089, 1090, 1091, \
    1092, 1093, 1094, 1095, 1095, 1096, 1097, 1098, 1099, 1100, 1101, 1102, 1102, 1103, 1104, 1105, 1106, 1107, 1108, 1109, \
    1110, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117, 1117, 1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125, 1125, 1126, \
    1127, 1128, 1129, 1130, 1131, 1132, 1132, 1133, 1134, 1135, 1136, 1137, 1138, 1139, 1139, 1140, 1141, 1142, 1143, 1144, \
    1145, 1146, 1147, 1147, 1148, 1149, 1150, 1151, 1152, 1153, 1154, 1154, 1155, 1156, 1157, 1158, 1159, 1160, 1161, 1161, \
    1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1176, 1177, 1178, 1179, \
    1180, 1181, 1182, 1183, 1184, 1184, 1185, 1186, 1187, 1188, 1189, 1190, 1191, 1191, 1192, 1193, 1194, 1195, 1196, 1197, \
    1198, 1198, 1199, 1200, 1201, 1202, 1203, 1204, 1205, 1206, 1206, 1207, 1208, 1209, 1210, 1211, 1212, 1213, 1213, 1214, \
    1215, 1216, 1217, 1218, 1219, 1220, 1220, 1221, 1222, 1223, 1224, 1225, 1226, 1227, 1227, 1228, 1229, 1230, 1231, 1232, \
    1233, 1234, 1235, 1235, 1236, 1237, 1238, 1239, 1240, 1241, 1242, 1242, 1243, 1244, 1245, 1246, 1247, 1248, 1249, 1249, \
    1250, 1251, 1252, 1253, 1254, 1255, 1256, 1256, 1257, 1258, 1259, 1260, 1261, 1262, 1263, 1263, 1264, 1265, 1266, 1267, \
    1268, 1269, 1270, 1271, 1271, 1272, 1273, 1274, 1275, 1276, 1277, 1278, 1278, 1279, 1280, 1281, 1282, 1283, 1284, 1285, \
    1285, 1286, 1287, 1288, 1289, 1290, 1291, 1292, 1292, 1293, 1294, 1295, 1296, 1297, 1298, 1299, 1300, 1300, 1301, 1302, \
    1303, 1304, 1305, 1306, 1307, 1307, 1308, 1309, 1310, 1311, 1312, 1313, 1314, 1314, 1315, 1316, 1317, 1318, 1319, 1320, \
    1321, 1321, 1322, 1323, 1324, 1325, 1326, 1327, 1328, 1329, 1329, 1330, 1331, 1332, 1333, 1334, 1335, 1336, 1336, 1337, \
    1338, 1339, 1340, 1341, 1342, 1343, 1343, 1344, 1345, 1346, 1347, 1348, 1349, 1350, 1350, 1351, 1352, 1353, 1354, 1355, \
    1356, 1357, 1358, 1358, 1359, 1360, 1361, 1362, 1363, 1364, 1365, 1365, 1366, 1367, 1368, 1369, 1370, 1371, 1372, 1372, \
    1373, 1374, 1375, 1376, 1377, 1378, 1379, 1379, 1380, 1381, 1382, 1383, 1384, 1385, 1386, 1386, 1387, 1388, 1389, 1390, \
    1391, 1392, 1393, 1394, 1394, 1395, 1396, 1397, 1398, 1399, 1400, 1401, 1401, 1402, 1403, 1404, 1405, 1406, 1407, 1408, \
    1408, 1409, 1410, 1411, 1412, 1413, 1414, 1415, 1415, 1416, 1417, 1418, 1419, 1420, 1420, 1421, 1422, 1423, 1423, 1424, \
    1425, 1425, 1426, 1427, 1428, 1428, 1429, 1430, 1431, 1431, 1432, 1433, 1433, 1434, 1435, 1436, 1436, 1437, 1438, 1438, \
    1439, 1440, 1441, 1441, 1442, 1443, 1444, 1444, 1445, 1446, 1446, 1447, 1448, 1449, 1449, 1450, 1451, 1451, 1452, 1453, \
    1454, 1454, 1455, 1456, 1457, 1457, 1458, 1459, 1459, 1460, 1461, 1462, 1462, 1463, 1464, 1464, 1465, 1466, 1467, 1467, \
    1468, 1469, 1470, 1470, 1471, 1472, 1472, 1473, 1474, 1475, 1475, 1476, 1477, 1477, 1478, 1479, 1480, 1480, 1481, 1482, \
    1483, 1483, 1484, 1485, 1485, 1486, 1487, 1488, 1488, 1489, 1490, 1490, 1491, 1492, 1493, 1493, 1494, 1495, 1496, 1496, \
    1497, 1498, 1498, 1499, 1500, 1501, 1501, 1502, 1503, 1503, 1504, 1505, 1506, 1506, 1507, 1508, 1509, 1509, 1510, 1511, \
    1511, 1512, 1513, 1514, 1514, 1515, 1516, 1516, 1517, 1518, 1519, 1519, 1520, 1521, 1522, 1522, 1523, 1524, 1524, 1525, \
    1526, 1527, 1527, 1528, 1529, 1529, 1530, 1531, 1532, 1532, 1533, 1534, 1535, 1535, 1536, 1537, 1537, 1538, 1539, 1540, \
    1540, 1541, 1542, 1542, 1543, 1544, 1545, 1545, 1546, 1547, 1548, 1548, 1549, 1550, 1550, 1551, 1552, 1553, 1553, 1554, \
    1555, 1555, 1556, 1557, 1558, 1558, 1559, 1560, 1561, 1561, 1562, 1563, 1563, 1564, 1565, 1566, 1566, 1567, 1568, 1568, \
    1569, 1569, 1570, 1571, 1571, 1572, 1572, 1573, 1574, 1574, 1575, 1575, 1576, 1577, 1577, 1578, 1578, 1579, 1580, 1580, \
    1581, 1581, 1582, 1582, 1583, 1584, 1584, 1585, 1585, 1586, 1587, 1587, 1588, 1588, 1589, 1590, 1590, 1591, 1591, 1592, \
    1593, 1593, 1594, 1594, 1595, 1596, 1596, 1597, 1597, 1598, 1599, 1599, 1600, 1600, 1601, 1602, 1602, 1603, 1603, 1604, \
    1605, 1605, 1606, 1606, 1607, 1607, 1608, 1609, 1609, 1610, 1610, 1611, 1612, 1612, 1613, 1613, 1614, 1615, 1615, 1616, \
    1616, 1617, 1618, 1618, 1619, 1619, 1620, 1621, 1621, 1622, 1622, 1623, 1624, 1624, 1625, 1625, 1626, 1627, 1627, 1628, \
    1628, 1629, 1630, 1630, 1631, 1631, 1632, 1633, 1633, 1634, 1634, 1635, 1635, 1636, 1637, 1637, 1638, 1638, 1639, 1640, \
    1640, 1641, 1641, 1642, 1643, 1643, 1644, 1644, 1645, 1646, 1646, 1647, 1647, 1648, 1649, 1649, 1650, 1650, 1651, 1652, \
    1652, 1653, 1653, 1654, 1655, 1655, 1656, 1656, 1657, 1658, 1658, 1659, 1659, 1660, 1660, 1661, 1662, 1662, 1663, 1663, \
    1664, 1665, 1665, 1666, 1666, 1667, 1668, 1668, 1669, 1669, 1670, 1671, 1671, 1672, 1672, 1673, 1674, 1674, 1675, 1675, \
    1676, 1677, 1677, 1678, 1678, 1679, 1680, 1680, 1681, 1681, 1682, 1683, 1683, 1684, 1684, 1685, 1686, 1686, 1687, 1687, \
    1688, 1688, 1689, 1690, 1690, 1691, 1691, 1692, 1693, 1693, 1694, 1694, 1695, 1696, 1696, 1697, 1697, 1698, 1699, 1699, \
    1700, 1700, 1701, 1702, 1702, 1703, 1703, 1704, 1705, 1705, 1706, 1706, 1707, 1708, 1708, 1709, 1709, 1710, 1711, 1711, \
    1712, 1712, 1713, 1713, 1714, 1715, 1715, 1716, 1716, 1717, 1718, 1718, 1719, 1719, 1720, 1721, 1721, 1722, 1722, 1723, \
    1724, 1724, 1725, 1725, 1726, 1727, 1727, 1728, 1728, 1729, 1730, 1730, 1731, 1731, 1732, 1733, 1733, 1734, 1734, 1735, \
    1736, 1736, 1737, 1737, 1738, 1739, 1739, 1740, 1740, 1741, 1741, 1742, 1743, 1743, 1744, 1744, 1745, 1746, 1746, 1747, \
    1747, 1748, 1749, 1749, 1750, 1750, 1751, 1752, 1752, 1753, 1753, 1754, 1755, 1755, 1756, 1756, 1757, 1758, 1758, 1759, \
    1759, 1760, 1761, 1761, 1762, 1762, 1763, 1764, 1764, 1765, 1765, 1766, 1766, 1767, 1768, 1768, 1769, 1769, 1770, 1771, \
    1771, 1772, 1772, 1773, 1774, 1774, 1775, 1775, 1776, 1777, 1777, 1778, 1778, 1779, 1779, 1780, 1780, 1781, 1782, 1782, \
    1783, 1784, 1784, 1785, 1786, 1786, 1787, 1788, 1788, 1789, 1790, 1790, 1791, 1792, 1792, 1793, 1794, 1794, 1795, 1796, \
    1796, 1797, 1797, 1798, 1799, 1799, 1800, 1801, 1801, 1802, 1803, 1803, 1804, 1805, 1805, 1806, 1807, 1807, 1808, 1809, \
    1809, 1810, 1810, 1811, 1812, 1812, 1813, 1814, 1814, 1815, 1816, 1816, 1817, 1818, 1818, 1819, 1820, 1820, 1821, 1822, \
    1822, 1823, 1823, 1824, 1824, 1825, 1825, 1826, 1826, 1827, 1827, 1828, 1828, 1829, 1829, 1830, 1830, 1831, 1831, 1832, \
    1833, 1834, 1834, 1835, 1835, 1836, 1836, 1837, 1837, 1838, 1838, 1839, 1839, 1840, 1840, 1841, 1841, 1842, 1842, 1843, \
    1844, 1845, 1845, 1846, 1846, 1847, 1847, 1848, 1848, 1849, 1849, 1850, 1850, 1851, 1851, 1852, 1852, 1853, 1853, 1854, \
    1855, 1855, 1856, 1857, 1857, 1858, 1859, 1859, 1860, 1861, 1861, 1862, 1862, 1863, 1863, 1864, 1864, 1865, 1865, 1866, \
    1867, 1867, 1868, 1868, 1869, 1869, 1870, 1870, 1871, 1871, 1872, 1872, 1873, 1873, 1874, 1874, 1875, 1875, 1876, 1876, \
    1877, 1877, 1878, 1878, 1879, 1879, 1880, 1880, 1881, 1881, 1882, 1882, 1883, 1883, 1884, 1884, 1885, 1885, 1886, 1886, \
    1887, 1888, 1888, 1889, 1889, 1890, 1890, 1891, 1891, 1892, 1892, 1893, 1893, 1894, 1894, 1895, 1895, 1896, 1896, 1897, \
    1898, 1898, 1899, 1899, 1900, 1900, 1901, 1901, 1902, 1902, 1903, 1903, 1904, 1904, 1905, 1905, 1906, 1906, 1907, 1907, \
    1908, 1909, 1909, 1910, 1910, 1911, 1911, 1912, 1912, 1913, 1913, 1914, 1914, 1915, 1915, 1916, 1916, 1917, 1917, 1918, \
    1919, 1920, 1920, 1921, 1921, 1922, 1922, 1923, 1923, 1924, 1924, 1925, 1925, 1926, 1926, 1927, 1927, 1928, 1928, 1929, \
    1930, 1931, 1931, 1932, 1932, 1933, 1933, 1934, 1934, 1935, 1935, 1936, 1936, 1937, 1937, 1938, 1938, 1939, 1939, 1940, \
    1941, 1942, 1942, 1943, 1943, 1944, 1944, 1945, 1945, 1946, 1946, 1947, 1947, 1948, 1948, 1949, 1949, 1950, 1950, 1951, \
    1952, 1953, 1953, 1954, 1955, 1955, 1956, 1957, 1957, 1958, 1959, 1959, 1960, 1961, 1961, 1962, 1963, 1963, 1964, 1965, \
    1966, 1966, 1967, 1967, 1968, 1968, 1969, 1969, 1970, 1970, 1971, 1971, 1972, 1972, 1973, 1973, 1974, 1974, 1975, 1975, \
    1976, 1977, 1978, 1978, 1979, 1980, 1980, 1981, 1982, 1982, 1983, 1984, 1984, 1985, 1986, 1986, 1987, 1988, 1988, 1989, \
    1990, 1990, 1991, 1992, 1992, 1993, 1994, 1994, 1995, 1996, 1996, 1997, 1998, 1998, 1999, 2000, 2000, 2001, 2002, 2002, \
    2003, 2003, 2004, 2005, 2005, 2006, 2007, 2007, 2008, 2009, 2009, 2010, 2011, 2011, 2012, 2013, 2013, 2014, 2015, 2015, \
    2016, 2016, 2017, 2017, 2018, 2018, 2019, 2019, 2020, 2020, 2021, 2021, 2021, 2021, 2021, 2021, 2022, 2022, 2022, 2022, \
    2022, 2022, 2023, 2023, 2023, 2023, 2023, 2023, 2024, 2024, 2024, 2024, 2024, 2024, 2025, 2025, 2025, 2025, 2025, 2025, \
    2026, 2026, 2026, 2026, 2026, 2026, 2027, 2027, 2027, 2027, 2027, 2027, 2028, 2028, 2028, 2028, 2028, 2028, 2029, 2029, \
    2029, 2029, 2029, 2029, 2030, 2030, 2030, 2030, 2030, 2030, 2031, 2031, 2031, 2031, 2031, 2031, 2032, 2032, 2032, 2032, \
    2032, 2032, 2033, 2033, 2033, 2033, 2033, 2033, 2034, 2034, 2034, 2034, 2034, 2034, 2035, 2035, 2035, 2035, 2035, 2035, \
    2036, 2036, 2036, 2036, 2036, 2036, 2037, 2037, 2037, 2037, 2037, 2037, 2038, 2038, 2038, 2038, 2038, 2038, 2039, 2039, \
    2039, 2039, 2039, 2039, 2040, 2040, 2040, 2040, 2040, 2040, 2041, 2041, 2041, 2041, 2041, 2041, 2042, 2042, 2042, 2042, \
    2042, 2042, 2043, 2043, 2043, 2043, 2043, 2043, 2044, 2044, 2044, 2044, 2044, 2044, 2045, 2045, 2045, 2045, 2045, 2046, \
    2046, 2046, 2046, 2046, 2047, 2047, 2047, 2047
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
        {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 80, {} },
//    {0x4F, 1, {0x01} },
//    {REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table init_setting_vdo[] = {
    //GIP timing
    {0xFF, 3,{0x98,0x81,0x01}},
    {0x00, 1,{0x43}},
    {0x01, 1,{0x50}},
    {0x02, 1,{0x08}},
    {0x03, 1,{0x8D}},
    {0x04, 1,{0x00}},
    {0x05, 1,{0x00}},
    {0x06, 1,{0x00}},
    {0x07, 1,{0x00}},
    {0x08, 1,{0x83}},
    {0x09, 1,{0x04}},
    {0x0a, 1,{0x30}},
    {0x0b, 1,{0x03}},
    {0x0c, 1,{0x88}},
    {0x0D, 1,{0x88}},
    {0x0e, 1,{0x8A}},
    {0x0f, 1,{0x8A}},
    {0x31, 1,{0x2A}},
    {0x32, 1,{0x07}},
    {0x33, 1,{0x00}},
    {0x34, 1,{0x07}},
    {0x35, 1,{0x02}},
    {0x36, 1,{0x01}},
    {0x37, 1,{0x07}},
    {0x38, 1,{0x12}},
    {0x39, 1,{0x10}},
    {0x3a, 1,{0x13}},
    {0x3b, 1,{0x11}},
    {0x3c, 1,{0x08}},
    {0x3d, 1,{0x2B}},
    {0x3e, 1,{0x02}},
    {0x3f, 1,{0x02}},
    {0x40, 1,{0x07}},
    {0x41, 1,{0x07}},
    {0x42, 1,{0x02}},
    {0x43, 1,{0x02}},
    {0x44, 1,{0x07}},
    {0x45, 1,{0x06}},
    {0x46, 1,{0x06}},
    {0x47, 1,{0x2A}},
    {0x48, 1,{0x07}},
    {0x49, 1,{0x00}},
    {0x4a, 1,{0x07}},
    {0x4b, 1,{0x02}},
    {0x4c, 1,{0x01}},
    {0x4d, 1,{0x07}},
    {0x4e, 1,{0x13}},
    {0x4f, 1,{0x11}},
    {0x50, 1,{0x10}},
    {0x51, 1,{0x12}},
    {0x52, 1,{0x09}},
    {0x53, 1,{0x2B}},
    {0x54, 1,{0x07}},
    {0x55, 1,{0x02}},
    {0x56, 1,{0x07}},
    {0x57, 1,{0x07}},
    {0x58, 1,{0x02}},
    {0x59, 1,{0x02}},
    {0x5a, 1,{0x07}},
    {0x5b, 1,{0x06}},
    {0x5c, 1,{0x06}},
    {0xB0, 1,{0x33}},
    {0xB1, 1,{0x33}},
    {0xB2, 1,{0x00}},
    {0xBA, 1,{0x20}},
    {0xd0, 1,{0x01}},
    {0xd1, 1,{0x00}},
    {0xDF, 1,{0x87}},
    {0xE0, 1,{0x1E}},
    {0xe2, 1,{0x00}},
    {0xe6, 1,{0x22}},
    {0xE7, 1,{0x54}},

    {0xFF, 3,{0x98,0x81,0x02}},
    {0x4B, 1,{0x5A}},
    {0x4D, 1,{0x4E}},
    {0x1A, 1,{0x48}},
    {0x4E, 1,{0x00}},
    {0x70, 1,{0x34}},
    {0x73, 1,{0x0A}},
    {0x79, 1,{0x06}},

    {0xFF, 3,{0x98,0x81,0x03}},
    {0x83, 1,{0x30}},
    {0x84, 1,{0x00}},

    {0xFF, 3,{0x98,0x81,0x05}},
    {0x03, 1,{0x00}},
    {0x04, 1,{0x7E}},
    {0x50, 1,{0x1F}},
    {0x58, 1,{0x61}},
    {0x63, 1,{0x8D}},
    {0x64, 1,{0x8D}},
    {0x68, 1,{0x29}},
    {0x69, 1,{0x43}},
    {0x6A, 1,{0x3D}},
    {0x6B, 1,{0x43}},

    {0xFF, 3,{0x98,0x81,0x06}},
    {0xDD, 1,{0x10}},
    {0x11, 1,{0x03}},
    {0x13, 1,{0x15}},
    {0x14, 1,{0x41}},
    {0x15, 1,{0xc2}},
    {0x16, 1,{0x40}},
    {0x17, 1,{0x48}},
    {0x18, 1,{0x3B}},
    {0xD6, 1,{0x87}},
    {0x27, 1,{0x20}},
    {0x28, 1,{0x20}},
    {0x2E, 1,{0x01}},
    {0xC0, 1,{0xF7}},
    {0xC1, 1,{0x02}},
    {0xC2, 1,{0x04}},
    {0x48, 1,{0x0F}},
    {0x4D, 1,{0x80}},
    {0x4E, 1,{0x40}},
    {0x7C, 1,{0x40}},
    {0x06, 1,{0xC4}},
    {0x94, 1,{0x00}},

    {0xFF, 3,{0x98,0x81,0x07}},
    {0x0F, 1,{0x02}},

    {0xFF, 3,{0x98,0x81,0x08}},
    {0xE0,27,{0x00,0x24,0x67,0x99,0xD9,0x55,0x0F,0x38,0x68,0x8F,0xA5,0xCC,0xFC,0x27,0x50,0xAA,0x7C,0xB2,0xD3,0xFC,0xFF,0x1E,0x49,0x7D,0xA7,0x03,0xEC}},
    {0xE1,27,{0x00,0x24,0x67,0x99,0xD9,0x55,0x0F,0x38,0x68,0x8F,0xA5,0xCC,0xFC,0x27,0x50,0xAA,0x7C,0xB2,0xD3,0xFC,0xFF,0x1E,0x49,0x7D,0xA7,0x03,0xEC}},

    {0xFF, 3,{0x98,0x81,0x0E}},
    {0x00, 1,{0xA0}},
    {0x11, 1,{0x90}},
    {0x13, 1,{0x05}},

    //UI mode
    {0xFF, 3,{0x98,0x81,0x03}},
    {0x90, 1,{0x14}},
    {0x91, 1,{0xF6}},
    {0x92, 1,{0x16}},
    {0x93, 1,{0xF6}},
    {0xAD, 1,{0xF8}},
    {0x8D, 1,{0x81}},
    //Still mode
    {0xFF, 3,{0x98,0x81,0x03}},
    {0x94, 1,{0x12}},
    {0x95, 1,{0x13}},
    {0x96, 1,{0x13}},
    {0x97, 1,{0x13}},
    {0x98, 1,{0x14}},
    {0x99, 1,{0x14}},
    {0x9a, 1,{0x14}},
    {0x9b, 1,{0x15}},
    {0x9C, 1,{0x15}},
    {0x9D, 1,{0x15}},
    {0xAE, 1,{0xED}},
    {0x8E, 1,{0x01}},
    //Moving mode
    {0xFF, 3,{0x98,0x81,0x03}},
    {0x9E, 1,{0x10}},
    {0x9F, 1,{0x11}},
    {0xA0, 1,{0x12}},
    {0xA1, 1,{0x12}},
    {0xA2, 1,{0x14}},
    {0xA3, 1,{0x14}},
    {0xA4, 1,{0x14}},
    {0xA5, 1,{0x14}},
    {0xA6, 1,{0x95}},
    {0xA7, 1,{0x95}},
    {0xAF, 1,{0xE7}},
    {0x8F, 1,{0x05}},

    {0xFF, 3,{0x98,0x81,0x00}},
    //{0x51, 2,{0x0F,0xFE}},
    {0x53, 1,{0x24}},
    {0x55, 1,{0x01}},
    {0x68, 2,{0x02,0x01}},
    {0x35, 1,{0x00}},
    {0x11, 1,{0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29, 1,{0x00}},
    {REGFLAG_DELAY, 20, {}}
};

static struct LCM_setting_table bl_level[] = {
    {0x51, 2, {0x0F,0xFF} },            //add by zhitonge
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

#ifdef VENDOR_EDIT
//Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Modified for 3level cabc feature.
static struct LCM_setting_table lcm_cabc_enter_setting_ui[] = {
    {0xFF, 3,{0x98,0x81,0x03}},
    {0xAC, 1,{0xFF}},
    {0x89, 1,{0xA0}},

    {0xFF, 3,{0x98,0x81,0x00}},
    {0x55, 1,{0x01}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_enter_setting_image[] = {
    {0xFF, 3,{0x98,0x81,0x03}},
    {0xAC, 1,{0xF9}},
    {0x89, 1,{0xA0}},

    {0xFF, 3,{0x98,0x81,0x00}},
    {0x55, 1,{0x02}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_enter_setting_video[] = {
    {0xFF, 3,{0x98,0x81,0x03}},
    {0xAC, 1,{0xFA}},
    {0x89, 1,{0xC0}},

    {0xFF, 3,{0x98,0x81,0x00}},
    {0x55, 1,{0x03}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#else
static struct LCM_setting_table lcm_cabc_enter_setting[] = {
    {0x55, 1, {0x01}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
    {0x55, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned int cmd;

    for (i = 0; i < count; i++) {
        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY:
            if (table[i].count <= 10)
                MDELAY(table[i].count);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_UDELAY:
            UDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V22(cmdq, cmd,
                table[i].count,
                table[i].para_list,
                force_update);
        }
    }
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->physical_width = LCM_PHYSICAL_WIDTH/1000;
    params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
    params->physical_width_um = LCM_PHYSICAL_WIDTH;
    params->physical_height_um = LCM_PHYSICAL_HEIGHT;
    params->density = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
    //lcm_dsi_mode = CMD_MODE;
#else
    params->dsi.mode = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
    //lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
    //LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
    params->dsi.switch_mode_enable = 0;

    /* DSI */
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_THREE_LANE;//LCM_FOUR_LANE;
    /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;
    /* video mode timing */
/*BITRATE 500M
FOUR_LANE
VIDEO_PIXEL_FORMAT 24bpp

RESOLUTIONXY 720,1440
PORCHSETTING VSYNCP,VBP,VFP,HSYNCP,HBP,HFP
PORCHSETTING 2,16,38,50,80,80
FRAME RATE 60 HZ

POWER SETTING
VSP 6
VSN -6
IOVCC 1.8*/
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 5;
    params->dsi.vertical_backporch = 16;
    params->dsi.vertical_frontporch = 232;
    //params->dsi.vertical_frontporch_for_low_power = 540;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 10;
    params->dsi.horizontal_backporch = 40;
    params->dsi.horizontal_frontporch = 34;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 365;
#else
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 360;
#endif
    //params->dsi.PLL_CK_CMD = 220;
    //params->dsi.PLL_CK_VDO = 255;
#else
    params->dsi.pll_div1 = 0;
    params->dsi.pll_div2 = 0;
    params->dsi.fbk_div = 0x1;
#endif
    /* clk continuous video mode */
    params->dsi.cont_clock = 0;

    params->dsi.CLK_HS_PRPR= 6;

    params->dsi.clk_lp_per_line_enable = 0;
#ifdef ATO_ESD_CHECK_DISABLE
    LCM_LOGI("%s,il9881 esd close\n", __func__);
    params->dsi.esd_check_enable = 0;
    params->dsi.customization_esd_check_enable = 0;
#else
    LCM_LOGI("%s,il9881 esd open\n", __func__);
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
#endif
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#if 0//def 0//CONFIG_MTK_ROUND_CORNER_SUPPORT
    params->round_corner_en = 1;
    params->full_content = 0;
    params->corner_pattern_width = 720;
    params->corner_pattern_height = 32;
    params->corner_pattern_height_bot = 32;
#endif
#ifndef BUILD_LK
    register_device_proc("lcd", "ili9881_hx", "txd vdo mode");
#endif
}

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static void lcm_init(void)
{
    int size;
//bias start
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
    int is_vsp_enabled = 0;
    int is_vsn_enabled = 0;

    is_vsp_enabled = pmi_lcd_bias_vsp_is_enabled();
    is_vsn_enabled = pmi_lcd_bias_vsn_is_enabled();
    LCM_LOGD("lcm_get_bias_status:%d %d\n", is_vsp_enabled, is_vsn_enabled);

    if(!is_vsp_enabled && !is_vsn_enabled) {
        LCM_LOGD("lcm_resume_power\n");
        SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5800);  //open lcd bias
        MDELAY(15);
    }
#endif
//bias end
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);

    SET_RESET_PIN(1);
    MDELAY(10);
    if(ilitek_tp)
        lcd_resume_load_ili_fw();

    size = sizeof(init_setting_vdo) /
        sizeof(struct LCM_setting_table);
    push_table(NULL, init_setting_vdo, size, 1);

#ifdef VENDOR_EDIT
//Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Add for 3level cabc feature.
    printk("set the cabc_lastlevel = %d\n",cabc_lastlevel);
	switch (cabc_lastlevel) {
        case 2 :
            push_table(NULL, lcm_cabc_enter_setting_image, sizeof(lcm_cabc_enter_setting_image) / sizeof(struct LCM_setting_table), 1);
            break;
        case 3 :
            push_table(NULL, lcm_cabc_enter_setting_video, sizeof(lcm_cabc_enter_setting_video) / sizeof(struct LCM_setting_table), 1);
            break;
        default :
            push_table(NULL, lcm_cabc_enter_setting_ui, sizeof(lcm_cabc_enter_setting_ui) / sizeof(struct LCM_setting_table), 1);
            break;
    }
#endif
}

static void lcm_suspend(void)
{
    LCM_LOGD("lcm_suspend\n");
    push_table(NULL, lcm_suspend_setting,
        sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
        1);
    MDELAY(10);

#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
    if(g_gesture <= 0) {
        LCM_LOGD("lcm_suspend_power\n");
        SET_LCD_BIAS_EN(OFF, VSN_FIRST_VSP_AFTER, 5800);  //open lcd bias
        MDELAY(15);
    }
#endif
    //SET_RESET_PIN(0);

}

static void lcm_resume(void)
{
    LCM_LOGD("lcm_resume\n");

    lcm_init();
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, version_id = 0;
    unsigned char buffer[2];
    unsigned int array[16];
    int ret = 0;
    //int lcm_id = 2;
    int lcd_id = 0;

    struct LCM_setting_table switch_table_page1[] = {
        { 0xFF, 0x03, {0x98, 0x81, 0x06} }
    };
    struct LCM_setting_table switch_table_page0[] = {
        { 0xFF, 0x03, {0x98, 0x81, 0x00} }
    };

//bias start
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
        SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5800);  //open lcd bias
        MDELAY(15);
#endif
//bias end

#if 1 // for lcm bias_on&hw_reset before mipi dsi goes to LP11    liyan 20180809
    SET_RESET_PIN(1);
    MDELAY(10);

    SET_RESET_PIN(0);
    MDELAY(10);

    SET_RESET_PIN(1);
    MDELAY(10);
#endif

    push_table(NULL,
        switch_table_page1,
        sizeof(switch_table_page1) / sizeof(struct LCM_setting_table),
        1);

    array[0] = 0x00023700;    /* read id return two byte,version and id */
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xf0, buffer, 1);
    id = buffer[0];        /* we only need ID */

    read_reg_v2(0xf1, buffer, 1);
    version_id = buffer[0];

    LCM_LOGI("%s,ili9881c_id=0x%08x,version_id=0x%x\n",__func__, id, version_id);
    printf("%s,ili9881c_id=0x%08x,version_id=0x%x\n",__func__, id, version_id);// id=0x00000098,version_id=0x81
    push_table(NULL,
        switch_table_page0,
        sizeof(switch_table_page0) / sizeof(struct LCM_setting_table),
        1);
    /*
    mt_set_gpio_mode(21, GPIO_MODE_00);//GPIO_LCM_ID
    mt_set_gpio_dir(21, GPIO_DIR_IN);
    lcm_id = mt_get_gpio_in(21);

    printf("xiaolong ili9881 lcm_id = %d\n",lcm_id);
    if (1 == lcm_id)
        return 1;
    else
        return 0;
        */
    //if (id == LCM_ID && version_id == 0x81)

    lcd_id = hq_get_lcd_module_id();

    if(0x15 == lcd_id)
        return 1;
    else
        return 0;

}
#endif

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
    LCM_LOGI("%s,ili9881c backlight: level = %d\n", __func__, level);

    if(level > 2047){
        level = 2047;
    }else if(level < 10 && level > 0 && EXPONENTIAL_REMAPPING != 1 ){
        level = 10;
    }
    if(EXPONENTIAL_REMAPPING){
        bl_level[0].para_list[0] = backlight_ili9881h_buf[level] >> 7;
        bl_level[0].para_list[1] = (backlight_ili9881h_buf[level] << 1) & 0xFE;
    }
    else{
        if(level < 18 && level > 0)
            level = 18;
        bl_level[0].para_list[0] = level >> 7;
        bl_level[0].para_list[1] = (level << 1) & 0xFE;
    }

    push_table(handle,
        bl_level,
        sizeof(bl_level) / sizeof(struct LCM_setting_table),
        1);

    esd_recovery_backlight_level = level; /* restore backlight level for esd recovery */
}

static void lcm_setbacklight(unsigned int level)
{
    LCM_LOGI("%s,il9881h backlight: level = %d\n", __func__, level);

    bl_level[0].para_list[0] = level;

    push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

#ifdef VENDOR_EDIT
//Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Modified for 3level cabc feature.
static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
	printk("%s [lcd] cabc_mode is %d ,lastlevel is %d \n", __func__, level,cabc_lastlevel);
	switch (level) {
        case 1 :
            push_table(handle,lcm_cabc_enter_setting_ui, sizeof(lcm_cabc_enter_setting_ui) / sizeof(struct LCM_setting_table), 1);
            break;
        case 2 :
            push_table(handle,lcm_cabc_enter_setting_image, sizeof(lcm_cabc_enter_setting_image) / sizeof(struct LCM_setting_table), 1);
            break;
        case 3 :
            push_table(handle,lcm_cabc_enter_setting_video, sizeof(lcm_cabc_enter_setting_video) / sizeof(struct LCM_setting_table), 1);
            break;
        default :
            push_table(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
            break;
        }
        if (level > 0) {
            cabc_lastlevel = level;
        }
}
#else
static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
    printk("%s [lcd] cabc_mode is %d \n", __func__, level);
    if (level) {
        push_table(handle,lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
    } else {
        push_table(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
    }
}
#endif

LCM_DRIVER ili9881_hd_dsi_vdo_txd_csot_zal1810_lcm_drv = {
    .name = "ili9881_txd_csot",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
#ifdef BUILD_LK
    .compare_id = lcm_compare_id,

#endif
    .init_power = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
    .set_backlight_cmdq = lcm_setbacklight_cmdq,
    .set_backlight = lcm_setbacklight,
    .set_cabc_mode_cmdq = lcm_set_cabc_mode_cmdq,
};
