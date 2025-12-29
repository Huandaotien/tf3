/*
 * tf3_c_api.h
 * C API wrapper for tf3 BufferCore to enable P/Invoke from C#
 *
 * Based on libtf2 tf2_c_api.
 */

#ifndef TF3_C_API_H
#define TF3_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Opaque handle for BufferCore */
typedef void* TF3_BufferCore;

/* Transform structure compatible with C# */
typedef struct {
    /* Header */
    int64_t timestamp_sec;
    int64_t timestamp_nsec;
    char frame_id[256];
    char child_frame_id[256];
    
    /* Translation */
    double translation_x;
    double translation_y;
    double translation_z;
    
    /* Rotation (quaternion) */
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
} TF3_Transform;

/* Error codes */
typedef enum {
    TF3_OK = 0,
    TF3_ERROR_LOOKUP = 1,
    TF3_ERROR_CONNECTIVITY = 2,
    TF3_ERROR_EXTRAPOLATION = 3,
    TF3_ERROR_INVALID_ARGUMENT = 4,
    TF3_ERROR_TIMEOUT = 5,
    TF3_ERROR_UNKNOWN = 99
} TF3_ErrorCode;

/* Export macro for shared library */
#ifdef _WIN32
    #ifdef TF3_C_API_EXPORTS
        #define TF3_C_API __declspec(dllexport)
    #else
        #define TF3_C_API __declspec(dllimport)
    #endif
#else
    #define TF3_C_API __attribute__((visibility("default")))
#endif

TF3_C_API TF3_BufferCore tf3_buffer_create(int32_t cache_time_sec);
TF3_C_API void tf3_buffer_destroy(TF3_BufferCore buffer);

TF3_C_API bool tf3_set_transform(
    TF3_BufferCore buffer,
    const TF3_Transform* transform,
    const char* authority,
    bool is_static
);

TF3_C_API bool tf3_lookup_transform(
    TF3_BufferCore buffer,
    const char* target_frame,
    const char* source_frame,
    int64_t time_sec,
    int64_t time_nsec,
    TF3_Transform* transform,
    TF3_ErrorCode* error_code
);

TF3_C_API bool tf3_lookup_transform_full(
    TF3_BufferCore buffer,
    const char* target_frame,
    int64_t target_time_sec,
    int64_t target_time_nsec,
    const char* source_frame,
    int64_t source_time_sec,
    int64_t source_time_nsec,
    const char* fixed_frame,
    TF3_Transform* transform,
    TF3_ErrorCode* error_code
);

TF3_C_API bool tf3_can_transform(
    TF3_BufferCore buffer,
    const char* target_frame,
    const char* source_frame,
    int64_t time_sec,
    int64_t time_nsec,
    char* error_msg,
    int32_t error_msg_len
);

TF3_C_API int32_t tf3_get_all_frame_names(
    TF3_BufferCore buffer,
    char* frames,
    int32_t frames_len
);

TF3_C_API bool tf3_get_frame_tree(
    TF3_BufferCore buffer,
    char* output,
    int32_t output_len
);

TF3_C_API void tf3_clear(TF3_BufferCore buffer);
TF3_C_API void tf3_get_current_time(int64_t* sec, int64_t* nsec);

TF3_C_API bool tf3_get_last_error(
    TF3_BufferCore buffer,
    char* error_msg,
    int32_t error_msg_len
);

TF3_C_API const char* tf3_get_version(void);

#ifdef __cplusplus
}
#endif

#endif /* TF3_C_API_H */
