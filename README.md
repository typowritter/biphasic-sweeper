## 工程说明

2017年全国大学生电子设计竞赛（H题软件部分）

题目要求：[远程幅频特性测试装置](Doc/远程幅频特性测试装置（H题）.pdf)

[Github](https://github.com/typowritter/biphasic-sweeper)

## 使用模块

开发板：野火STM32H743_Pro（[资料链接](http://doc.embedfire.com/products/link/zh/latest/stm32/ebf_stm32h743_pro/download/stm32h743_pro.html)）

DDS：AD9854（[评估板链接](https://detail.tmall.com/item.htm?id=552781984800)）

ADC：ADS124S08（[TI裸片](https://www.ti.com/product/ADS124S08) | [TI评估板](https://www.ti.com/tool/ADS124S08EVM)）

TFT组态屏：广州大彩DC10600GM070_1111_1T（[官网](http://www.gz-dc.com/)）

## 开发环境

| 软件                | 作用           | 版本                       |
| :------------------ | -------------- | -------------------------- |
| STM32CubeMX         | STM32工程配置  | 6.2.1                      |
| STM32CubeMX 固件包  | -              | STM32Cube_FW_H7_V1.8.0     |
| STM32CubeProgrammer | 程序下载、调试 | 2.7.0                      |
| gcc-arm-none-eabi   | 程序编译       | 15:6.3.1+svn253039-1build1 |
| minicom             | 串口调试       | 2.7.1                      |
| VisualTFT           | 大彩串口屏开发 | 3.0.0.1185                 |

## 工程结构说明

| 目录       | 说明                                                         |
| ---------- | ------------------------------------------------------------ |
| Core/      | 用户代码                                                     |
| Drivers/   | STM32库文件（由CubeMX生成）                                  |
| Doc/       | 设计文档、开发记录等（以Git Submodule形式提供，[源地址](https://github.com/typowritter/eedesign-doc)） |
| VisualTFT/ | 大彩串口屏工程                                               |

