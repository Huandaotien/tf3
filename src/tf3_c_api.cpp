/*
 * tf3_c_api.cpp
 * C API wrapper implementation for tf3 BufferCore
 */

#include "tf3/tf3_c_api.h"
#include "tf3/buffer_core.h"
#include "tf3/exceptions.h"
#include "tf3/time.h"
#include <string>
#include <cstring>
#include <chrono>
#include <map>
#include <mutex>
#include <vector>

// Thread-safe error storage
static std::mutex error_mutex;
static std::map<TF3_BufferCore, std::string> error_messages;

// Helper to store error message
static void store_error(TF3_BufferCore buffer, const std::string& msg) {
    std::lock_guard<std::mutex> lock(error_mutex);
    error_messages[buffer] = msg;
}

// Helper to convert TF3_Transform to TransformStampedMsg
static tf3::TransformStampedMsg to_cpp_transform(const TF3_Transform* t) {
    tf3::TransformStampedMsg msg;
    
    // Header - tf3::Time uses uint32_t sec/nsec
    msg.header.stamp.sec = static_cast<uint32_t>(t->timestamp_sec);
    msg.header.stamp.nsec = static_cast<uint32_t>(t->timestamp_nsec);
    msg.header.frame_id = t->frame_id;
    msg.child_frame_id = t->child_frame_id;
    
    // Translation
    msg.transform.translation.x = t->translation_x;
    msg.transform.translation.y = t->translation_y;
    msg.transform.translation.z = t->translation_z;
    
    // Rotation
    msg.transform.rotation.x = t->rotation_x;
    msg.transform.rotation.y = t->rotation_y;
    msg.transform.rotation.z = t->rotation_z;
    msg.transform.rotation.w = t->rotation_w;
    
    return msg;
}

// Helper to convert TransformStampedMsg to TF3_Transform
static void from_cpp_transform(const tf3::TransformStampedMsg& msg, TF3_Transform* t) {
    // Header - tf3::Time uses uint32_t sec/nsec
    t->timestamp_sec = static_cast<int64_t>(msg.header.stamp.sec);
    t->timestamp_nsec = static_cast<int64_t>(msg.header.stamp.nsec);
    
    strncpy(t->frame_id, msg.header.frame_id.c_str(), 255);
    t->frame_id[255] = '\0';
    strncpy(t->child_frame_id, msg.child_frame_id.c_str(), 255);
    t->child_frame_id[255] = '\0';
    
    // Translation
    t->translation_x = msg.transform.translation.x;
    t->translation_y = msg.transform.translation.y;
    t->translation_z = msg.transform.translation.z;
    
    // Rotation
    t->rotation_x = msg.transform.rotation.x;
    t->rotation_y = msg.transform.rotation.y;
    t->rotation_z = msg.transform.rotation.z;
    t->rotation_w = msg.transform.rotation.w;
}

// Helper to convert exception to error code
static TF3_ErrorCode exception_to_error_code(const std::exception& e) {
    if (dynamic_cast<const tf3::LookupException*>(&e)) {
        return TF3_ERROR_LOOKUP;
    } else if (dynamic_cast<const tf3::ConnectivityException*>(&e)) {
        return TF3_ERROR_CONNECTIVITY;
    } else if (dynamic_cast<const tf3::ExtrapolationException*>(&e)) {
        return TF3_ERROR_EXTRAPOLATION;
    } else if (dynamic_cast<const tf3::InvalidArgumentException*>(&e)) {
        return TF3_ERROR_INVALID_ARGUMENT;
    } else if (dynamic_cast<const tf3::TimeoutException*>(&e)) {
        return TF3_ERROR_TIMEOUT;
    }
    return TF3_ERROR_UNKNOWN;
}

extern "C" {

TF3_BufferCore tf3_buffer_create(int32_t cache_time_sec) {
    try {
        if (cache_time_sec <= 0) {
            cache_time_sec = 10; // Default
        }
        
        auto* buffer = new tf3::BufferCore(
            tf3::Duration(cache_time_sec, 0)
        );
        return static_cast<TF3_BufferCore>(buffer);
    } catch (...) {
        return nullptr;
    }
}

void tf3_buffer_destroy(TF3_BufferCore buffer) {
    if (!buffer) return;
    
    try {
        // Remove error message
        {
            std::lock_guard<std::mutex> lock(error_mutex);
            error_messages.erase(buffer);
        }
        
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        delete cpp_buffer;
    } catch (...) {
        // Ignore errors during destruction
    }
}

bool tf3_set_transform(
    TF3_BufferCore buffer,
    const TF3_Transform* transform,
    const char* authority,
    bool is_static
) {
    if (!buffer || !transform) return false;
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        auto cpp_transform = to_cpp_transform(transform);
        
        std::string auth = authority ? authority : "c_api";
        
        return cpp_buffer->setTransform(cpp_transform, auth, is_static);
    } catch (const std::exception& e) {
        store_error(buffer, e.what());
        return false;
    } catch (...) {
        store_error(buffer, "Unknown error in tf3_set_transform");
        return false;
    }
}

bool tf3_lookup_transform(
    TF3_BufferCore buffer,
    const char* target_frame,
    const char* source_frame,
    int64_t time_sec,
    int64_t time_nsec,
    TF3_Transform* transform,
    TF3_ErrorCode* error_code
) {
    if (!buffer || !target_frame || !source_frame || !transform) {
        if (error_code) *error_code = TF3_ERROR_INVALID_ARGUMENT;
        return false;
    }
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        
        tf3::Time time;
        if (time_sec == 0 && time_nsec == 0) {
            time = tf3::Time();
        } else {
            time.sec = static_cast<uint32_t>(time_sec);
            time.nsec = static_cast<uint32_t>(time_nsec);
        }
        
        auto result = cpp_buffer->lookupTransform(
            std::string(target_frame),
            std::string(source_frame),
            time
        );
        
        from_cpp_transform(result, transform);
        
        if (error_code) *error_code = TF3_OK;
        return true;
        
    } catch (const std::exception& e) {
        store_error(buffer, e.what());
        if (error_code) *error_code = exception_to_error_code(e);
        return false;
    } catch (...) {
        store_error(buffer, "Unknown error in tf3_lookup_transform");
        if (error_code) *error_code = TF3_ERROR_UNKNOWN;
        return false;
    }
}

bool tf3_lookup_transform_full(
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
) {
    if (!buffer || !target_frame || !source_frame || !fixed_frame || !transform) {
        if (error_code) *error_code = TF3_ERROR_INVALID_ARGUMENT;
        return false;
    }
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        
        tf3::Time target_time;
        target_time.sec = static_cast<uint32_t>(target_time_sec);
        target_time.nsec = static_cast<uint32_t>(target_time_nsec);
        
        tf3::Time source_time;
        source_time.sec = static_cast<uint32_t>(source_time_sec);
        source_time.nsec = static_cast<uint32_t>(source_time_nsec);
        
        auto result = cpp_buffer->lookupTransform(
            std::string(target_frame),
            target_time,
            std::string(source_frame),
            source_time,
            std::string(fixed_frame)
        );
        
        from_cpp_transform(result, transform);
        
        if (error_code) *error_code = TF3_OK;
        return true;
        
    } catch (const std::exception& e) {
        store_error(buffer, e.what());
        if (error_code) *error_code = exception_to_error_code(e);
        return false;
    } catch (...) {
        store_error(buffer, "Unknown error in tf3_lookup_transform_full");
        if (error_code) *error_code = TF3_ERROR_UNKNOWN;
        return false;
    }
}

bool tf3_can_transform(
    TF3_BufferCore buffer,
    const char* target_frame,
    const char* source_frame,
    int64_t time_sec,
    int64_t time_nsec,
    char* error_msg,
    int32_t error_msg_len
) {
    if (!buffer || !target_frame || !source_frame) {
        return false;
    }
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        
        tf3::Time time;
        if (time_sec == 0 && time_nsec == 0) {
            time = tf3::Time();
        } else {
            time.sec = static_cast<uint32_t>(time_sec);
            time.nsec = static_cast<uint32_t>(time_nsec);
        }
        
        std::string error_string;
        bool result = cpp_buffer->canTransform(
            std::string(target_frame),
            std::string(source_frame),
            time,
            error_msg ? &error_string : nullptr
        );
        
        if (!result && error_msg && error_msg_len > 0) {
            strncpy(error_msg, error_string.c_str(), error_msg_len - 1);
            error_msg[error_msg_len - 1] = '\0';
        }
        
        return result;
        
    } catch (...) {
        return false;
    }
}

int32_t tf3_get_all_frame_names(
    TF3_BufferCore buffer,
    char* frames,
    int32_t frames_len
) {
    if (!buffer || !frames || frames_len <= 0) {
        return 0;
    }
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        std::vector<std::string> frame_vec;
        cpp_buffer->_getFrameStrings(frame_vec);
        
        std::string result;
        for (const auto& frame : frame_vec) {
            if (!result.empty()) result += "\n";
            result += frame;
        }
        
        strncpy(frames, result.c_str(), frames_len - 1);
        frames[frames_len - 1] = '\0';
        
        return static_cast<int32_t>(frame_vec.size());
        
    } catch (...) {
        return 0;
    }
}

bool tf3_get_frame_tree(
    TF3_BufferCore buffer,
    char* output,
    int32_t output_len
) {
    if (!buffer || !output || output_len <= 0) {
        return false;
    }
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        std::string tree = cpp_buffer->allFramesAsString();
        
        strncpy(output, tree.c_str(), output_len - 1);
        output[output_len - 1] = '\0';
        
        return true;
        
    } catch (...) {
        return false;
    }
}

void tf3_clear(TF3_BufferCore buffer) {
    if (!buffer) return;
    
    try {
        auto* cpp_buffer = static_cast<tf3::BufferCore*>(buffer);
        cpp_buffer->clear();
    } catch (...) {
        // Ignore
    }
}

void tf3_get_current_time(int64_t* sec, int64_t* nsec) {
    if (!sec || !nsec) return;
    
    try {
        auto now = tf3::Time::now();
        *sec = static_cast<int64_t>(now.sec);
        *nsec = static_cast<int64_t>(now.nsec);
    } catch (...) {
        *sec = 0;
        *nsec = 0;
    }
}

bool tf3_get_last_error(
    TF3_BufferCore buffer,
    char* error_msg,
    int32_t error_msg_len
) {
    if (!buffer || !error_msg || error_msg_len <= 0) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(error_mutex);
    auto it = error_messages.find(buffer);
    
    if (it != error_messages.end() && !it->second.empty()) {
        strncpy(error_msg, it->second.c_str(), error_msg_len - 1);
        error_msg[error_msg_len - 1] = '\0';
        return true;
    }
    
    return false;
}

const char* tf3_get_version(void) {
    return "libtf3 v1.0 C API";
}

} // extern "C"
