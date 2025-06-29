// Copyright © 2016, 2019, Oracle and/or its affiliates. All rights reserved.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License, version 2.0, as
// published by the Free Software Foundation.
//
// This program is also distributed with certain software (including
// but not limited to OpenSSL) that is licensed under separate terms,
// as designated in a particular file or component or in included license
// documentation.  The authors of MySQL hereby grant you an
// additional permission to link the program and your derivative works
// with the separately licensed software that they have included with
// MySQL.
//
// Without limiting anything contained in the foregoing, this file,
// which is part of MySQL Connector/NET, is also subject to the
// Universal FOSS Exception, version 1.0, a copy of which can be found at
// http://oss.oracle.com/licenses/universal-foss-exception.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License, version 2.0, for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin St, Fifth Floor, Boston, MA 02110-1301  USA

using MySql.Data;
using System;
using System.Collections.Generic;

namespace MySqlX.Data
{
  internal class CollationMap
  {
    private static Dictionary<int, string> collations = new Dictionary<int, string>();

    static CollationMap()
    {
      Load();
    }

    public static string GetCollationName(int collation)
    {
      if (!collations.ContainsKey(collation))
        throw new KeyNotFoundException(String.Format(ResourcesX.InvalidCollationId, collation));
      return collations[collation];
    }

    private static void Load()
    {
      collations.Add(0, "utf8mb3_general_ci");
      collations.Add(1, "big5_chinese_ci");
      collations.Add(2, "latin2_czech_cs");
      collations.Add(3, "dec8_swedish_ci");
      collations.Add(4, "cp850_general_ci");
      collations.Add(5, "latin1_german1_ci");
      collations.Add(6, "hp8_english_ci");
      collations.Add(7, "koi8r_general_ci");
      collations.Add(8, "latin1_swedish_ci");
      collations.Add(9, "latin2_general_ci");
      collations.Add(10, "swe7_swedish_ci");
      collations.Add(11, "ascii_general_ci");
      collations.Add(12, "ujis_japanese_ci");
      collations.Add(13, "sjis_japanese_ci");
      collations.Add(14, "cp1251_bulgarian_ci");
      collations.Add(15, "latin1_danish_ci");
      collations.Add(16, "hebrew_general_ci");
      collations.Add(18, "tis620_thai_ci");
      collations.Add(19, "euckr_korean_ci");
      collations.Add(20, "latin7_estonian_cs");
      collations.Add(21, "latin2_hungarian_ci");
      collations.Add(22, "koi8u_general_ci");
      collations.Add(23, "cp1251_ukrainian_ci");
      collations.Add(24, "gb2312_chinese_ci");
      collations.Add(25, "greek_general_ci");
      collations.Add(26, "cp1250_general_ci");
      collations.Add(27, "latin2_croatian_ci");
      collations.Add(28, "gbk_chinese_ci");
      collations.Add(29, "cp1257_lithuanian_ci");
      collations.Add(30, "latin5_turkish_ci");
      collations.Add(31, "latin1_german2_ci");
      collations.Add(32, "armscii8_general_ci");
      collations.Add(33, "utf8mb3_general_ci");
      collations.Add(34, "cp1250_czech_cs");
      collations.Add(35, "ucs2_general_ci");
      collations.Add(36, "cp866_general_ci");
      collations.Add(37, "keybcs2_general_ci");
      collations.Add(38, "macce_general_ci");
      collations.Add(39, "macroman_general_ci");
      collations.Add(40, "cp852_general_ci");
      collations.Add(41, "latin7_general_ci");
      collations.Add(42, "latin7_general_cs");
      collations.Add(43, "macce_bin");
      collations.Add(44, "cp1250_croatian_ci");
      collations.Add(45, "utf8mb4_general_ci");
      collations.Add(46, "utf8mb4_bin");
      collations.Add(47, "latin1_bin");
      collations.Add(48, "latin1_general_ci");
      collations.Add(49, "latin1_general_cs");
      collations.Add(50, "cp1251_bin");
      collations.Add(51, "cp1251_general_ci");
      collations.Add(52, "cp1251_general_cs");
      collations.Add(53, "macroman_bin");
      collations.Add(54, "utf16_general_ci");
      collations.Add(55, "utf16_bin");
      collations.Add(56, "utf16le_general_ci");
      collations.Add(57, "cp1256_general_ci");
      collations.Add(58, "cp1257_bin");
      collations.Add(59, "cp1257_general_ci");
      collations.Add(60, "utf32_general_ci");
      collations.Add(61, "utf32_bin");
      collations.Add(62, "utf16le_bin");
      collations.Add(63, "binary");
      collations.Add(64, "armscii8_bin");
      collations.Add(65, "ascii_bin");
      collations.Add(66, "cp1250_bin");
      collations.Add(67, "cp1256_bin");
      collations.Add(68, "cp866_bin");
      collations.Add(69, "dec8_bin");
      collations.Add(70, "greek_bin");
      collations.Add(71, "hebrew_bin");
      collations.Add(72, "hp8_bin");
      collations.Add(73, "keybcs2_bin");
      collations.Add(74, "koi8r_bin");
      collations.Add(75, "koi8u_bin");
      collations.Add(76, "utf8mb3_tolower_ci");
      collations.Add(77, "latin2_bin");
      collations.Add(78, "latin5_bin");
      collations.Add(79, "latin7_bin");
      collations.Add(80, "cp850_bin");
      collations.Add(81, "cp852_bin");
      collations.Add(82, "swe7_bin");
      collations.Add(83, "utf8mb3_bin");
      collations.Add(84, "big5_bin");
      collations.Add(85, "euckr_bin");
      collations.Add(86, "gb2312_bin");
      collations.Add(87, "gbk_bin");
      collations.Add(88, "sjis_bin");
      collations.Add(89, "tis620_bin");
      collations.Add(90, "ucs2_bin");
      collations.Add(91, "ujis_bin");
      collations.Add(92, "geostd8_general_ci");
      collations.Add(93, "geostd8_bin");
      collations.Add(94, "latin1_spanish_ci");
      collations.Add(95, "cp932_japanese_ci");
      collations.Add(96, "cp932_bin");
      collations.Add(97, "eucjpms_japanese_ci");
      collations.Add(98, "eucjpms_bin");
      collations.Add(99, "cp1250_polish_ci");
      collations.Add(101, "utf16_unicode_ci");
      collations.Add(102, "utf16_icelandic_ci");
      collations.Add(103, "utf16_latvian_ci");
      collations.Add(104, "utf16_romanian_ci");
      collations.Add(105, "utf16_slovenian_ci");
      collations.Add(106, "utf16_polish_ci");
      collations.Add(107, "utf16_estonian_ci");
      collations.Add(108, "utf16_spanish_ci");
      collations.Add(109, "utf16_swedish_ci");
      collations.Add(110, "utf16_turkish_ci");
      collations.Add(111, "utf16_czech_ci");
      collations.Add(112, "utf16_danish_ci");
      collations.Add(113, "utf16_lithuanian_ci");
      collations.Add(114, "utf16_slovak_ci");
      collations.Add(115, "utf16_spanish2_ci");
      collations.Add(116, "utf16_roman_ci");
      collations.Add(117, "utf16_persian_ci");
      collations.Add(118, "utf16_esperanto_ci");
      collations.Add(119, "utf16_hungarian_ci");
      collations.Add(120, "utf16_sinhala_ci");
      collations.Add(121, "utf16_german2_ci");
      collations.Add(122, "utf16_croatian_ci");
      collations.Add(123, "utf16_unicode_520_ci");
      collations.Add(124, "utf16_vietnamese_ci");
      collations.Add(128, "ucs2_unicode_ci");
      collations.Add(129, "ucs2_icelandic_ci");
      collations.Add(130, "ucs2_latvian_ci");
      collations.Add(131, "ucs2_romanian_ci");
      collations.Add(132, "ucs2_slovenian_ci");
      collations.Add(133, "ucs2_polish_ci");
      collations.Add(134, "ucs2_estonian_ci");
      collations.Add(135, "ucs2_spanish_ci");
      collations.Add(136, "ucs2_swedish_ci");
      collations.Add(137, "ucs2_turkish_ci");
      collations.Add(138, "ucs2_czech_ci");
      collations.Add(139, "ucs2_danish_ci");
      collations.Add(140, "ucs2_lithuanian_ci");
      collations.Add(141, "ucs2_slovak_ci");
      collations.Add(142, "ucs2_spanish2_ci");
      collations.Add(143, "ucs2_roman_ci");
      collations.Add(144, "ucs2_persian_ci");
      collations.Add(145, "ucs2_esperanto_ci");
      collations.Add(146, "ucs2_hungarian_ci");
      collations.Add(147, "ucs2_sinhala_ci");
      collations.Add(148, "ucs2_german2_ci");
      collations.Add(149, "ucs2_croatian_ci");
      collations.Add(150, "ucs2_unicode_520_ci");
      collations.Add(151, "ucs2_vietnamese_ci");
      collations.Add(159, "ucs2_general_mysql500_ci");
      collations.Add(160, "utf32_unicode_ci");
      collations.Add(161, "utf32_icelandic_ci");
      collations.Add(162, "utf32_latvian_ci");
      collations.Add(163, "utf32_romanian_ci");
      collations.Add(164, "utf32_slovenian_ci");
      collations.Add(165, "utf32_polish_ci");
      collations.Add(166, "utf32_estonian_ci");
      collations.Add(167, "utf32_spanish_ci");
      collations.Add(168, "utf32_swedish_ci");
      collations.Add(169, "utf32_turkish_ci");
      collations.Add(170, "utf32_czech_ci");
      collations.Add(171, "utf32_danish_ci");
      collations.Add(172, "utf32_lithuanian_ci");
      collations.Add(173, "utf32_slovak_ci");
      collations.Add(174, "utf32_spanish2_ci");
      collations.Add(175, "utf32_roman_ci");
      collations.Add(176, "utf32_persian_ci");
      collations.Add(177, "utf32_esperanto_ci");
      collations.Add(178, "utf32_hungarian_ci");
      collations.Add(179, "utf32_sinhala_ci");
      collations.Add(180, "utf32_german2_ci");
      collations.Add(181, "utf32_croatian_ci");
      collations.Add(182, "utf32_unicode_520_ci");
      collations.Add(183, "utf32_vietnamese_ci");
      collations.Add(192, "utf8mb3_unicode_ci");
      collations.Add(193, "utf8mb3_icelandic_ci");
      collations.Add(194, "utf8mb3_latvian_ci");
      collations.Add(195, "utf8mb3_romanian_ci");
      collations.Add(196, "utf8mb3_slovenian_ci");
      collations.Add(197, "utf8mb3_polish_ci");
      collations.Add(198, "utf8mb3_estonian_ci");
      collations.Add(199, "utf8mb3_spanish_ci");
      collations.Add(200, "utf8mb3_swedish_ci");
      collations.Add(201, "utf8mb3_turkish_ci");
      collations.Add(202, "utf8mb3_czech_ci");
      collations.Add(203, "utf8mb3_danish_ci");
      collations.Add(204, "utf8mb3_lithuanian_ci");
      collations.Add(205, "utf8mb3_slovak_ci");
      collations.Add(206, "utf8mb3_spanish2_ci");
      collations.Add(207, "utf8mb3_roman_ci");
      collations.Add(208, "utf8mb3_persian_ci");
      collations.Add(209, "utf8mb3_esperanto_ci");
      collations.Add(210, "utf8mb3_hungarian_ci");
      collations.Add(211, "utf8mb3_sinhala_ci");
      collations.Add(212, "utf8mb3_german2_ci");
      collations.Add(213, "utf8mb3_croatian_ci");
      collations.Add(214, "utf8mb3_unicode_520_ci");
      collations.Add(215, "utf8mb3_vietnamese_ci");
      collations.Add(223, "utf8mb3_general_mysql500_ci");
      collations.Add(224, "utf8mb4_unicode_ci");
      collations.Add(225, "utf8mb4_icelandic_ci");
      collations.Add(226, "utf8mb4_latvian_ci");
      collations.Add(227, "utf8mb4_romanian_ci");
      collations.Add(228, "utf8mb4_slovenian_ci");
      collations.Add(229, "utf8mb4_polish_ci");
      collations.Add(230, "utf8mb4_estonian_ci");
      collations.Add(231, "utf8mb4_spanish_ci");
      collations.Add(232, "utf8mb4_swedish_ci");
      collations.Add(233, "utf8mb4_turkish_ci");
      collations.Add(234, "utf8mb4_czech_ci");
      collations.Add(235, "utf8mb4_danish_ci");
      collations.Add(236, "utf8mb4_lithuanian_ci");
      collations.Add(237, "utf8mb4_slovak_ci");
      collations.Add(238, "utf8mb4_spanish2_ci");
      collations.Add(239, "utf8mb4_roman_ci");
      collations.Add(240, "utf8mb4_persian_ci");
      collations.Add(241, "utf8mb4_esperanto_ci");
      collations.Add(242, "utf8mb4_hungarian_ci");
      collations.Add(243, "utf8mb4_sinhala_ci");
      collations.Add(244, "utf8mb4_german2_ci");
      collations.Add(245, "utf8mb4_croatian_ci");
      collations.Add(246, "utf8mb4_unicode_520_ci");
      collations.Add(247, "utf8mb4_vietnamese_ci");
      collations.Add(248, "gb18030_chinese_ci");
      collations.Add(249, "gb18030_bin");
      collations.Add(250, "gb18030_unicode_520_ci");
      collations.Add(255, "utf8mb4_0900_ai_ci");
      collations.Add(256, "utf8mb4_de_pb_0900_ai_ci");
      collations.Add(257, "utf8mb4_is_0900_ai_ci");
      collations.Add(258, "utf8mb4_lv_0900_ai_ci");
      collations.Add(259, "utf8mb4_ro_0900_ai_ci");
      collations.Add(260, "utf8mb4_sl_0900_ai_ci");
      collations.Add(261, "utf8mb4_pl_0900_ai_ci");
      collations.Add(262, "utf8mb4_et_0900_ai_ci");
      collations.Add(263, "utf8mb4_es_0900_ai_ci");
      collations.Add(264, "utf8mb4_sv_0900_ai_ci");
      collations.Add(265, "utf8mb4_tr_0900_ai_ci");
      collations.Add(266, "utf8mb4_cs_0900_ai_ci");
      collations.Add(267, "utf8mb4_da_0900_ai_ci");
      collations.Add(268, "utf8mb4_lt_0900_ai_ci");
      collations.Add(269, "utf8mb4_sk_0900_ai_ci");
      collations.Add(270, "utf8mb4_es_trad_0900_ai_ci");
      collations.Add(271, "utf8mb4_la_0900_ai_ci");
      collations.Add(273, "utf8mb4_eo_0900_ai_ci");
      collations.Add(274, "utf8mb4_hu_0900_ai_ci");
      collations.Add(275, "utf8mb4_hr_0900_ai_ci");
      collations.Add(277, "utf8mb4_vi_0900_ai_ci");
      collations.Add(278, "utf8mb4_0900_as_cs");
      collations.Add(279, "utf8mb4_de_pb_0900_as_cs");
      collations.Add(280, "utf8mb4_is_0900_as_cs");
      collations.Add(281, "utf8mb4_lv_0900_as_cs");
      collations.Add(282, "utf8mb4_ro_0900_as_cs");
      collations.Add(283, "utf8mb4_sl_0900_as_cs");
      collations.Add(284, "utf8mb4_pl_0900_as_cs");
      collations.Add(285, "utf8mb4_et_0900_as_cs");
      collations.Add(286, "utf8mb4_es_0900_as_cs");
      collations.Add(287, "utf8mb4_sv_0900_as_cs");
      collations.Add(288, "utf8mb4_tr_0900_as_cs");
      collations.Add(289, "utf8mb4_cs_0900_as_cs");
      collations.Add(290, "utf8mb4_da_0900_as_cs");
      collations.Add(291, "utf8mb4_lt_0900_as_cs");
      collations.Add(292, "utf8mb4_sk_0900_as_cs");
      collations.Add(293, "utf8mb4_es_trad_0900_as_cs");
      collations.Add(294, "utf8mb4_la_0900_as_cs");
      collations.Add(296, "utf8mb4_eo_0900_as_cs");
      collations.Add(297, "utf8mb4_hu_0900_as_cs");
      collations.Add(298, "utf8mb4_hr_0900_as_cs");
      collations.Add(300, "utf8mb4_vi_0900_as_cs");
      collations.Add(303, "utf8mb4_ja_0900_as_cs");
      collations.Add(304, "utf8mb4_ja_0900_as_cs_ks");
      collations.Add(305, "utf8mb4_0900_as_ci");
      collations.Add(306, "utf8mb4_ru_0900_ai_ci");
      collations.Add(307, "utf8mb4_ru_0900_as_cs");
      collations.Add(308, "utf8mb4_zh_0900_as_cs");
      collations.Add(309, "utf8mb4_0900_bin");
      collations.Add(310, "utf8mb4_nb_0900_ai_ci");
      collations.Add(311, "utf8mb4_nb_0900_as_cs");
      collations.Add(312, "utf8mb4_nn_0900_ai_ci");
      collations.Add(313, "utf8mb4_nn_0900_as_cs");
      collations.Add(314, "utf8mb4_sr_latn_0900_ai_ci");
      collations.Add(315, "utf8mb4_sr_latn_0900_as_cs");
      collations.Add(316, "utf8mb4_bs_0900_ai_ci");
      collations.Add(317, "utf8mb4_bs_0900_as_cs");
      collations.Add(318, "utf8mb4_bg_0900_ai_ci");
      collations.Add(319, "utf8mb4_bg_0900_as_cs");
      collations.Add(320, "utf8mb4_gl_0900_ai_ci");
      collations.Add(321, "utf8mb4_gl_0900_as_cs");
      collations.Add(322, "utf8mb4_mn_cyrl_0900_ai_ci");
      collations.Add(323, "utf8mb4_mn_cyrl_0900_as_cs");
    }
  }
}
