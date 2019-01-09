/*******************************************************************************
 * File Name    :	Version.h
 * File Describe:	版本声明及与版本相关定义
 * Relate Module:	Version.h
 * Writer		:   
 * Date			:   2018-10-12
**********************************************************************************/
#ifndef __VERSION_H_
#define __VERSION_H_

#define _STR(s)             #s					    //# 运算符能将宏的参数转换为带双引号的字符串，
#define STR(s)              _STR(s)

/* EI-Device version information */
#define EI_VERSION                      1              /**< major version number */
#define EI_SUBVERSION                   0              /**< minor version number */
#define EI_REVISION                     0              /**< revise version number */

/* EI-Device version */
#define EI_DEVICE_VERSION               ((EI_VERSION * 10000) + \
                                        (EI_SUBVERSION * 100) + EI_REVISION)
                                         
#define EI_SOFT_VER 										"Ver:"STR(EI_VERSION)"."STR(EI_SUBVERSION)"."STR(EI_REVISION)

void ei_show_version(void);

#endif




