#pragma once

#include "rapidjson/document.h"
#include <fstream>
#ifdef WIN32
#include "windows.h"
#include "libloaderapi.h"
#include "shlwapi.h"
#endif // WIN32
#include <sstream>
#include <string>

#ifdef WIN32
#define PATH_LENGTH MAX_PATH
#define PATH_SEPARATOR "\\"
#else
#define PATH_LENGTH 500
#define PATH_SEPARATOR "/"
#endif

#define CONFIG_FILE_NAME "fledermaus_config.json"

#define SPEED_NAME Speed
#define FIST_TO_LIFT_NAME FistToLiftActive
#define RIGHT_CLICK_ACTIVE_NAME RightClickActive
#define LOCK_MOUSE_ON_SCROLL_NAME LockMouseOnScroll
#define SCROLLING_ON_NAME ScrollingActive
#define SCROLLING_SPEED_NAME ScrollingSpeed
#define SCROLL_THRESHOLD_NAME ScrollThreshold
#define ORIENTATION_NAME VerticalOrientation
#define INDEX_PINCH_THRESHOLD_NAME IndexPinchThreshold
#define USE_ABSOLUTE_MOUSE_POSITION UseAbsoluteMousePosition
#define BOUNDS_LEFT_NAME BoundsLeftMeters
#define BOUNDS_RIGHT_NAME BoundsRightMeters
#define BOUNDS_LOWER_NAME BoundsLowerMeters
#define BOUNDS_UPPER_NAME BoundsUpperMeters
#define BOUNDS_NEAR_NAME BoundsNearMeters
#define BOUNDS_FAR_NAME BoundsFarMeters
#define LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME LimitTrackingToWithinBounds
#define LEAP_CAMERA_MODE TrackingMode
#define HANDEDNESS Handedness

#define STRINGIFY(x) #x
#define STRINGIFY_HELPER(x) STRINGIFY(x)
#define TOKENPASTE_HELPER(x, y) x ## y
#define TOKENPASTE(x, y) TOKENPASTE_HELPER(x, y)

namespace rjs=rapidjson;


#define SETTERS_AND_GETTERS_BOOL(name, def) private: \
    bool TOKENPASTE(name, _) = def; \
    public: \
    void TOKENPASTE(Set, name)(const bool name) \
    { \
        TOKENPASTE(name, _) = name; \
    } \
    bool TOKENPASTE(Get, name) () const \
    { \
        return TOKENPASTE(name, _); \
    }

#define SETTERS_AND_GETTERS_FLOAT(name, default) private: \
    float TOKENPASTE(name, _) = default; \
    public: \
    void TOKENPASTE(Set, name)(const float name) \
    { \
        TOKENPASTE(name, _) = name; \
    } \
    float TOKENPASTE(Get, name) () const \
    { \
        return TOKENPASTE(name, _); \
    }

#define SETTERS_AND_GETTERS_STRING(name, default) private: \
    std::string TOKENPASTE(name, _) = default; \
    public: \
    void TOKENPASTE(Set, name)(const std::string& name) \
    { \
        TOKENPASTE(name, _) = name; \
    } \
    std::string TOKENPASTE(Get, name) () const \
    { \
        return TOKENPASTE(name, _); \
    }

class ConfigReader {

    SETTERS_AND_GETTERS_FLOAT(SPEED_NAME, 2.0f);
    SETTERS_AND_GETTERS_BOOL(SCROLLING_ON_NAME, true);
    SETTERS_AND_GETTERS_FLOAT(SCROLLING_SPEED_NAME, 4.0f);
    SETTERS_AND_GETTERS_FLOAT(SCROLL_THRESHOLD_NAME, 20.0f)
    SETTERS_AND_GETTERS_BOOL(ORIENTATION_NAME, true);
    SETTERS_AND_GETTERS_BOOL(LOCK_MOUSE_ON_SCROLL_NAME, true);
    SETTERS_AND_GETTERS_BOOL(RIGHT_CLICK_ACTIVE_NAME, true);
    SETTERS_AND_GETTERS_BOOL(FIST_TO_LIFT_NAME, true);
    SETTERS_AND_GETTERS_FLOAT(INDEX_PINCH_THRESHOLD_NAME, 35.0f);
    SETTERS_AND_GETTERS_BOOL(USE_ABSOLUTE_MOUSE_POSITION, false);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_LEFT_NAME, 0.25f);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_RIGHT_NAME, 0.25f);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_LOWER_NAME, 0.10f);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_UPPER_NAME, 0.35f);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_NEAR_NAME, 0.15f);
    SETTERS_AND_GETTERS_FLOAT(BOUNDS_FAR_NAME, 0.15f);
    SETTERS_AND_GETTERS_BOOL(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME, false);
    SETTERS_AND_GETTERS_STRING(LEAP_CAMERA_MODE, "desktop");
    SETTERS_AND_GETTERS_STRING(HANDEDNESS, "both");

    private:
    std::string config_file_name_;
    rjs::Document d_;

    public:
    ConfigReader(std::string config_file_name) :
    config_file_name_(config_file_name)
    {
        init();
    }

    ConfigReader()
    {
        char configFilePath[PATH_LENGTH];
        getExecutableDirectory(configFilePath, PATH_LENGTH); 
        std::stringstream ss;
        ss << configFilePath << PATH_SEPARATOR << CONFIG_FILE_NAME;
        config_file_name_ = ss.str();
        init();
    }

    ~ConfigReader() {}

    void print()
    {
        printf( STRINGIFY_HELPER(SPEED_NAME) ": %f\n", TOKENPASTE(SPEED_NAME, _));
        printf( STRINGIFY_HELPER(FIST_TO_LIFT_NAME) ": %s\n", TOKENPASTE(FIST_TO_LIFT_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(RIGHT_CLICK_ACTIVE_NAME) ": %s\n", TOKENPASTE(RIGHT_CLICK_ACTIVE_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(LOCK_MOUSE_ON_SCROLL_NAME) ": %s\n", TOKENPASTE(LOCK_MOUSE_ON_SCROLL_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(SCROLLING_ON_NAME) ": %s\n", TOKENPASTE(SCROLLING_ON_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(SCROLLING_SPEED_NAME) ": %f\n", TOKENPASTE(SCROLLING_SPEED_NAME, _));
        printf( STRINGIFY_HELPER(SCROLL_THRESHOLD_NAME) ": %f\n", TOKENPASTE(SCROLL_THRESHOLD_NAME, _));
        printf( STRINGIFY_HELPER(ORIENTATION_NAME) ": %s\n", TOKENPASTE(ORIENTATION_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(INDEX_PINCH_THRESHOLD_NAME) ": %f\n", TOKENPASTE(INDEX_PINCH_THRESHOLD_NAME, _));
        printf( STRINGIFY_HELPER(USE_ABSOLUTE_MOUSE_POSITION) ": %s\n", TOKENPASTE(USE_ABSOLUTE_MOUSE_POSITION, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(BOUNDS_LEFT_NAME) ": %f\n", TOKENPASTE(BOUNDS_LEFT_NAME, _));
        printf( STRINGIFY_HELPER(BOUNDS_RIGHT_NAME) ": %f\n", TOKENPASTE(BOUNDS_RIGHT_NAME, _));
        printf( STRINGIFY_HELPER(BOUNDS_LOWER_NAME) ": %f\n", TOKENPASTE(BOUNDS_LOWER_NAME, _));
        printf( STRINGIFY_HELPER(BOUNDS_UPPER_NAME) ": %f\n", TOKENPASTE(BOUNDS_UPPER_NAME, _));
        printf( STRINGIFY_HELPER(BOUNDS_NEAR_NAME) ": %f\n", TOKENPASTE(BOUNDS_NEAR_NAME, _));
        printf( STRINGIFY_HELPER(BOUNDS_FAR_NAME) ": %f\n", TOKENPASTE(BOUNDS_FAR_NAME, _));
        printf( STRINGIFY_HELPER(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME) ": %s\n", TOKENPASTE(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME, _) ? "true" : "false");
        printf( STRINGIFY_HELPER(LEAP_CAMERA_MODE) ": %s\n", TOKENPASTE(LEAP_CAMERA_MODE, _.c_str()));
        printf( STRINGIFY_HELPER(HANDEDNESS) ": %s\n", TOKENPASTE(HANDEDNESS, _.c_str()));
    }

    private:
    void init()
    {
        printf("Looking for config file at %s\n", config_file_name_.c_str());
        std::ifstream ifs(config_file_name_);

        if (!ifs.fail())
        {
            std::string line;
            std::stringstream ss;
            while (std::getline(ifs, line))
            {
                ss << line;
            }

            d_.Parse(ss.str().c_str());

            validateAndLoadJson(); 
        }
        else
        {
            printf("Error reading config file, using defaults.\n");
            return;
        }
    }

    static size_t getExecutableDirectory(char* dest, size_t destLength)
    {
#ifdef WIN32
    // This will break if we start dealing with unicode
    // TCHAR is a char if using ANSI, a wide if using unicode
    TCHAR path[PATH_LENGTH];
    DWORD length = GetModuleFileName(NULL, path, MAX_PATH);
    PathRemoveFileSpec(path);

    strncpy(dest, reinterpret_cast<const char*>(path), static_cast<size_t>(length));

    return static_cast<size_t>(length);
#endif // WIN32
    }

    void validateAndLoadJson()
    {
        if (d_.HasMember(STRINGIFY_HELPER(ORIENTATION_NAME )))
        {
            // assert(d_[STRINGIFY_HELPER(ORIENTATION_NAME)].IsString());
            TOKENPASTE(ORIENTATION_NAME, _) = d_[STRINGIFY_HELPER(ORIENTATION_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(ORIENTATION_NAME) " not found!\n");
        }
        
        if (d_.HasMember(STRINGIFY_HELPER(SPEED_NAME)))
        {
            // assert(d_[STRINGIFY(SPEED_NAME)].IsNumber());
            TOKENPASTE(SPEED_NAME, _) = d_[STRINGIFY_HELPER(SPEED_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(SPEED_NAME) " not found!\n");
        }
        
        if (d_.HasMember(STRINGIFY_HELPER(FIST_TO_LIFT_NAME)))
        {
            // assert(d_[STRINGIFY(SCROLLING_SPEED_NAME)].IsFloat());
            TOKENPASTE(FIST_TO_LIFT_NAME, _) = d_[STRINGIFY_HELPER(FIST_TO_LIFT_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(FIST_TO_LIFT_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(RIGHT_CLICK_ACTIVE_NAME)))
        {
            // assert(d_[STRINGIFY(RIGHT_CLICK_ACTIVE_NAME)].IsBool());
            TOKENPASTE(RIGHT_CLICK_ACTIVE_NAME, _) = d_[STRINGIFY_HELPER(RIGHT_CLICK_ACTIVE_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(RIGHT_CLICK_ACTIVE_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(LOCK_MOUSE_ON_SCROLL_NAME)))
        {
            // assert(d_[STRINGIFY(LOCK_MOUSE_ON_SCROLL_NAME)].IsBool());
            TOKENPASTE(LOCK_MOUSE_ON_SCROLL_NAME, _) = d_[STRINGIFY_HELPER(LOCK_MOUSE_ON_SCROLL_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(LOCK_MOUSE_ON_SCROLL_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(SCROLLING_ON_NAME)))
        {
            // assert(d_[STRINGIFY(SCROLLING_ON_NAME)].IsBool());
            TOKENPASTE(SCROLLING_ON_NAME, _) = d_[STRINGIFY_HELPER(SCROLLING_ON_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(SCROLLING_ON_NAME) " not found!\n");
        }
        
        if (d_.HasMember(STRINGIFY_HELPER(SCROLLING_SPEED_NAME)))
        {
            // assert(d_[STRINGIFY(SCROLLING_SPEED_NAME)].IsFloat());
            TOKENPASTE(SCROLLING_SPEED_NAME, _) = d_[STRINGIFY_HELPER(SCROLLING_SPEED_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(SCROLLING_SPEED_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(SCROLL_THRESHOLD_NAME)))
        {
            // assert(d_[STRINGIFY(SCROLL_THRESHOLD_NAME)].IsFloat());
            TOKENPASTE(SCROLL_THRESHOLD_NAME, _) = d_[STRINGIFY_HELPER(SCROLL_THRESHOLD_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(SCROLL_THRESHOLD_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(INDEX_PINCH_THRESHOLD_NAME)))
        {
            // assert(d_[STRINGIFY(INDEX_PINCH_THRESHOLD_NAME)].IsFloat());
            TOKENPASTE(INDEX_PINCH_THRESHOLD_NAME, _) = d_[STRINGIFY_HELPER(INDEX_PINCH_THRESHOLD_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(INDEX_PINCH_THRESHOLD_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(USE_ABSOLUTE_MOUSE_POSITION)))
        {
            // assert(d_[STRINGIFY(USE_ABSOLUTE_MOUSE_POSITION)].IsBool());
            TOKENPASTE(USE_ABSOLUTE_MOUSE_POSITION, _) = d_[STRINGIFY_HELPER(USE_ABSOLUTE_MOUSE_POSITION)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(USE_ABSOLUTE_MOUSE_POSITION) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_LEFT_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_WIDTH_LEFT_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_LEFT_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_LEFT_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_LEFT_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_RIGHT_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_WIDTH_RIGHT_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_RIGHT_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_RIGHT_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_RIGHT_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_LOWER_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_HEIGHT_LOWER_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_LOWER_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_LOWER_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_LOWER_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_UPPER_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_HEIGHT_UPPER_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_UPPER_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_UPPER_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_UPPER_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_NEAR_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_NEAR_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_NEAR_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_NEAR_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_NEAR_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(BOUNDS_FAR_NAME)))
        {
            // assert(d_[STRINGIFY(BOUNDS_FAR_NAME)].IsFloat());
            TOKENPASTE(BOUNDS_FAR_NAME, _) = d_[STRINGIFY_HELPER(BOUNDS_FAR_NAME)].GetFloat();
        }
        else
        {
            printf(STRINGIFY_HELPER(BOUNDS_FAR_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME)))
        {
            // assert(d_[STRINGIFY(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME)].IsBool());
            TOKENPASTE(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME, _) = d_[STRINGIFY_HELPER(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME)].GetBool();
        }
        else
        {
            printf(STRINGIFY_HELPER(LIMIT_TRACKING_TO_WITHIN_BOUNDS_NAME) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(LEAP_CAMERA_MODE)))
        {
            // assert(d_[STRINGIFY(LEAP_CAMERA_MODE)].IsBool());
            TOKENPASTE(LEAP_CAMERA_MODE, _) = d_[STRINGIFY_HELPER(LEAP_CAMERA_MODE)].GetString();
        }
        else
        {
            printf(STRINGIFY_HELPER(LEAP_CAMERA_MODE) " not found!\n");
        }

        if (d_.HasMember(STRINGIFY_HELPER(HANDEDNESS)))
        {
            // assert(d_[STRINGIFY(HANDEDNESS)].IsBool());
            TOKENPASTE(HANDEDNESS, _) = d_[STRINGIFY_HELPER(HANDEDNESS)].GetString();
        }
        else
        {
            printf(STRINGIFY_HELPER(HANDEDNESS) " not found!\n");
        }
    }
};
