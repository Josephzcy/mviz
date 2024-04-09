#ifndef MVIZ_COLLECT_INF_H
#define MVIZ_COLLECT_INF_H
//
// Created by nick on 23-2-13.
//

#include <iostream>
#include <vector>
// 3rdparty
#include "collect/collect_dbm.hpp"
#include "collect/collect_srv.hpp"
#include "glog/logging.h"



#if (WIN32)
#ifdef MVIZ_COLLECT_EXPORTS
#define MVIZ_COLLECT_API __declspec(dllexport)
#else
#define MVIZ_COLLECT_API __declspec(dllexport)
#endif
#else
#define MVIZ_COLLECT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif
MVIZ_COLLECT_API int mviz_collect_init(int);      // 初始化 mviz_record 功能
MVIZ_COLLECT_API int mviz_collect_start(void);    // 开始 mviz_record 功能
MVIZ_COLLECT_API int mviz_collect_stop(void);     // 对于 mviz_record 的初始化
MVIZ_COLLECT_API int mviz_collect_release(void);  // 释放 mviz_record 的功能
#ifdef __cplusplus
}
MVIZ_COLLECT_API int mviz_collect_init1();  // 初始化 mviz_record 功能
#endif

#endif  // MIV_COLLECT_INF_H
