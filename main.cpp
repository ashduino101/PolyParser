/***********************************************************************************************************************
 * Copyright (C) 2022 Ashton Fairchild (ashduino101). All rights reserved.
 * Licensed under the MIT License. See LICENSE in the project root for license information.
 *
 * This file is part of PolyParser.
 *
 * If you would like to contribute to the project, it is advised you collapse all in the file, as this file is thousands
 * of lines long.
 **********************************************************************************************************************/


// Macros
#define MAX_VERSION 26  // Maximum layout version fully supported
#define MAX_BRIDGE_VERSION 11  // Maximum bridge version fully supported
#define MAX_SLOT_VERSION 3  // Maximum slot version fully supported
#define MAX_PHYSICS_VERSION 1  // Maximum physics engine version fully supported

// Standard library
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <codecvt>

// People might have this
#include <getopt.h>

// 3rd-party libraries
#include "inc/json.hpp"
#include "inc/fifo_map.hpp"  // For ordered JSON

// #include <yaml.h>  TODO - YAML support

#include "inc/base64.h"

// JSON map workaround
template<class K, class V, class dummy_compare, class A>
using workaround_fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
using json = nlohmann::basic_json<workaround_fifo_map>;

bool silent = false;
int unusualNumbers = 1;

enum BridgeMaterialType {
    INVALID,
    ROAD,
    REINFORCED_ROAD,
    WOOD,
    STEEL,
    HYDRAULICS,
    ROPE,
    CABLE,
    BUNGINE_ROPE,
    SPRING
};
enum SplitJointPart {
    A,
    B,
    C
};
enum SplitJointState {
    ALL_SPLIT,
    NONE_SPLIT,
    A_SPLIT_ONLY,
    B_SPLIT_ONLY,
    C_SPLIT_ONLY,
};
enum StrengthMethod {
    Acceleration,
    MaxSlope,
    TorquePerWheel
};
enum TerrainIslandType {
    Bookend,
    Middle
};
enum SplineType {
    Hermite,
    BSpline,
    Bezier,
    Linear
};
// Save slot enums
enum BinaryEntryType {
    Invalid = 0x0,
    NamedStartOfReferenceNode = 0x1,
    UnnamedStartOfReferenceNode = 0x2,
    NamedStartOfStructNode = 0x3,
    UnnamedStartOfStructNode = 0x4,
    EndOfNode = 0x5,
    StartOfArray = 0x6,
    EndOfArray = 0x7,
    PrimitiveArray = 0x8,
    NamedInternalReference = 0x9,
    UnnamedInternalReference = 0xA,
    NamedExternalReferenceByIndex = 0xB,
    UnnamedExternalReferenceByIndex = 0xC,
    NamedExternalReferenceByGuid = 0xD,
    UnnamedExternalReferenceByGuid = 0xE,
    NamedSByte = 0xF,
    UnnamedSByte = 0x10,
    NamedByte = 0x11,
    UnnamedByte = 0x12,
    NamedShort = 0x13,
    UnnamedShort = 0x14,
    NamedUShort = 0x15,
    UnnamedUShort = 0x16,
    NamedInt = 0x17,
    UnnamedInt = 0x18,
    NamedUInt = 0x19,
    UnnamedUInt = 0x1A,
    NamedLong = 0x1B,
    UnnamedLong = 0x1C,
    NamedULong = 0x1D,
    UnnamedULong = 0x1E,
    NamedFloat = 0x1F,
    UnnamedFloat = 0x20,
    NamedDouble = 0x21,
    UnnamedDouble = 0x22,
    NamedDecimal = 0x23,
    UnnamedDecimal = 0x24,
    NamedChar = 0x25,
    UnnamedChar = 0x26,
    NamedString = 0x27,
    UnnamedString = 0x28,
    NamedGuid = 0x29,
    UnnamedGuid = 0x2A,
    NamedBoolean = 0x2B,
    UnnamedBoolean = 0x2C,
    NamedNull = 0x2D,
    UnnamedNull = 0x2E,
    TypeName = 0x2F,
    TypeID = 0x30,
    EndOfStream = 0x31,
    NamedExternalReferenceByString = 0x32,
    UnnamedExternalReferenceByString = 0x33
};
enum EntryType {
    InvalidType = 0x0,
    String = 0x1,
    Guid = 0x2,
    Integer = 0x3,
    FloatingPoint = 0x4,
    Boolean = 0x5,
    Null = 0x6,
    StartOfNode = 0x7,
    EndOfNodeType = 0x8,
    InternalReference = 0x9,
    ExternalReferenceByIndex = 0xA,
    ExternalReferenceByGuid = 0xB,
    StartOfArrayType = 0xC,
    EndOfArrayType = 0xD,
    PrimitiveArrayType = 0xE,
    EndOfStreamType = 0xF,
    ExternalReferenceByString = 0x10
};

struct Vec3 {
    float x, y, z;
};
struct Vec2 {
    float x, y;
};
struct Quaternion {
    float x, y, z, w;
};
struct Color {
    float r, g, b, a;
};
struct BridgeJoint {
    Vec3 pos{};
    bool is_anchor{};
    bool is_split{};
    std::string guid;
};
struct BridgeEdge {
    BridgeMaterialType material_type{};
    std::string node_a_guid;
    std::string node_b_guid;
    SplitJointPart joint_a_part{};
    SplitJointPart joint_b_part{};
    std::string guid;
};
struct BridgeSpring {
    float normalized_value{};
    std::string node_a_guid;
    std::string node_b_guid;
    std::string guid;
};
struct BridgeSplitJoint {
    std::string guid;
    SplitJointState state{};
};
struct Piston {
    float normalized_value{};  // Fixed when initialized.
    std::string node_a_guid;
    std::string node_b_guid;
    std::string guid;
};
struct HydraulicPhase {
    float time_delay{};
    std::string guid;
};
struct ZAxisVehicle {
    Vec2 pos{};
    std::string prefab_name;
    std::string guid;
    float time_delay{};
    float speed{};
    Quaternion rot{};
    float rotation_degrees{};
};
struct Vehicle {
    std::string display_name;
    Vec2 pos{};
    Quaternion rot{};
    std::string prefab_name;
    float target_speed{};
    float mass{};
    float braking_force_multiplier{};
    StrengthMethod strength_method{};
    float acceleration{};
    float max_slope{};
    float desired_acceleration{};
    float shocks_multiplier{};
    float rotation_degrees{};
    float time_delay{};
    bool idle_on_downhill{};
    bool flipped{};
    bool ordered_checkpoints{};
    std::string guid;
    std::vector<std::string> checkpoint_guids;
};
struct VehicleStopTrigger {
    Vec2 pos{};
    Quaternion rot{};
    float height{};
    float rotation_degrees{};
    bool flipped{};
    std::string prefab_name;
    std::string stop_vehicle_guid;
};
struct ThemeObject {
    Vec2 pos;
    std::string prefab_name;
    bool unknown_value;
};
struct EventUnit {
    std::string guid;
};
struct EventStage {
    std::vector<EventUnit> units;
};
struct EventTimeline {
    std::string checkpoint_guid;
    std::vector<EventStage> stages;
};
struct Checkpoint {
    Vec2 pos{};
    std::string prefab_name;
    std::string vehicle_guid;
    std::string vehicle_restart_phase_guid;
    bool trigger_timeline{};
    bool stop_vehicle{};
    bool reverse_vehicle_on_restart{};
    std::string guid;
};
struct Platform {
    Vec2 pos;
    float width;
    float height;
    bool flipped;
    bool solid;
};
struct HydraulicsControllerPhase {
    std::string hydraulics_phase_guid;
    std::vector<std::string> piston_guids;
    std::vector<BridgeSplitJoint> bridge_split_joints;
    bool disable_new_additions{};
};
struct TerrainIsland {
    Vec3 pos{};
    std::string prefab_name;
    float height_added{};
    float right_edge_water_height{};
    TerrainIslandType terrain_island_type{};
    int variant_index{};
    bool flipped{};
    bool lock_position{};
};
struct Ramp {
    Vec2 pos;
    std::vector<Vec2> control_points;
    float height;
    int num_segments;
    SplineType spline_type;
    bool flipped_vertical;
    bool flipped_horizontal;
    bool hide_legs;
    bool flipped_legs;
    std::vector<Vec2> line_points;
};
struct VehicleRestartPhase {
    float time_delay{};
    std::string guid;
    std::string vehicle_guid;
};
struct FlyingObject {
    Vec3 pos{};
    Vec3 scale{};
    std::string prefab_name;
};
struct Rock {
    Vec3 pos{};
    Vec3 scale{};
    std::string prefab_name;
    bool flipped;
};
struct WaterBlock {
    Vec3 pos{};
    float width{};
    float height{};
    bool lock_position{};
};
struct Bridge {
    int version{};
    std::vector<BridgeJoint> joints;
    std::vector<BridgeEdge> edges;
    std::vector<BridgeSpring> springs;
    std::vector<Piston> pistons;
    std::vector<BridgeJoint> anchors;
    std::vector<HydraulicsControllerPhase> phases;
};
struct Budget {
    int cash;
    int road;
    int wood;
    int steel;
    int hydraulics;
    int rope;
    int cable;
    int bungee_rope;
    int spring;
    bool allow_wood;
    bool allow_steel;
    bool allow_hydraulics;
    bool allow_rope;
    bool allow_cable;
    bool allow_spring;
    bool allow_reinforced_road;
};
struct Settings {
    bool hydraulics_controller_enabled;
    bool unbreakable;
};
struct CustomShape {
    Vec3 pos{};
    Quaternion rot{};
    Vec3 scale{};
    bool flipped{};
    bool dynamic{};
    bool collides_with_road{};
    bool collides_with_nodes{};
    bool collides_with_split_nodes{};  // v25+
    float rotation_degrees{};
    Color color{};  // v10+, otherwise deserialize int
    float mass{};  // v11+, otherwise deserialize float and set to 40f
    float bounciness{};  // v14+, otherwise set to 0.5f
    float pin_motor_strength{};  // v24+, otherwise set to 0f
    float pin_target_velocity{};  // ^ ^ ^
    std::vector<Vec2> points_local_space;
    std::vector<Vec3> static_pins;
    std::vector<std::string> dynamic_anchor_guids;
};
struct Workshop {
    std::string id;
    std::string leaderboard_id;
    std::string title;
    std::string description;
    bool autoplay{};
    std::vector<std::string> tags;
};
struct SupportPillar {
    Vec3 pos{};
    Vec3 scale{};
    std::string prefab_name;
};
struct Pillar {
    Vec3 pos{};
    float height{};
    std::string prefab_name;
};
// PTF support
struct Mod {
    std::string name;
    std::string version;
    std::string settings;
};
struct ModSaveData {
    char* data;
    std::string name;
    std::string version;
};
struct ModData {
    std::vector<Mod> mods;
    std::vector<ModSaveData> mod_save_data;
};
struct Layout {
    int32_t version{};
    std::string stubKey;
    std::vector<BridgeJoint> anchors;
    std::vector<HydraulicPhase> phases;
    Bridge bridge;
    std::vector<ZAxisVehicle> zAxisVehicles;
    std::vector<Vehicle> vehicles;
    std::vector<VehicleStopTrigger> vehicleStopTriggers;
    std::vector<ThemeObject> themeObjects_OBSOLETE;
    std::vector<EventTimeline> eventTimelines;
    std::vector<Checkpoint> checkpoints;
    std::vector<Platform> platforms;
    std::vector<TerrainIsland> terrainStretches;
    std::vector<Ramp> ramps;
    std::vector<VehicleRestartPhase> vehicleRestartPhases;
    std::vector<FlyingObject> flyingObjects;
    std::vector<Rock> rocks;
    std::vector<WaterBlock> waterBlocks;
    Budget budget{};
    Settings settings{};
    std::vector<CustomShape> customShapes;
    Workshop workshop{};
    std::vector<SupportPillar> supportPillars;
    std::vector<Pillar> pillars;
    bool isModded{};
    ModData modData;
};
// Save slot support
struct SaveSlot {
    int version{};
    int physicsVersion{};
    int slotId{};
    std::string displayName;
    std::string fileName;
    int budget{};
    long lastWriteTimeTicks{};
    Bridge bridge;
    bool unlimitedMaterials{};
    bool unlimitedBudget{};
    char* thumbnail{};
};
struct EntryTypeReturn {
    EntryType type;
    std::string name;
};
struct TypeEntryReturn {
    std::string typeName;
    std::string assemblyName;
};

namespace Utils {
    std::string prettyPrintStubKeyToTheme(const std::string &stubKey) {
        if (stubKey == "PineMountains") {
            return "\x1B[38;5;28mPine Mountains\x1B[0m";  // greenish
        } else if (stubKey == "Volcano") {
            return "\x1B[38;5;202mGlowing Gorge\x1B[0m";  // kind of orange
        } else if (stubKey == "Savanna") {
            return "\x1B[38;5;214mTranquil Oasis\x1B[0m";  // golden brown kind of
        } else if (stubKey == "Western") {
            return "\x1B[38;5;220mSanguine Gulch\x1B[0m";  // light yellow
        } else if (stubKey == "ZenGardens") {
            return "\x1B[38;5;163mSerenity Valley\x1B[0m";  // purplish
        } else if (stubKey == "Steampunk") {
            return "\x1B[38;5;94mSteamtown\x1B[0m";  // brown
        } else {
            return "\x1B[38;5;196mINVALID\x1B[0m";  // red
        }
    }

    // TODO: maybe use a bool and 3 functions instead of 9 functions and a bunch of templates
    // These functions are unsafe, but shouldn't be used elsewhere, so it should be fine.
    template<typename ...Args>
    void log_info_d(const std::string& message, Args ...args) {
        if (!silent) {
            // noinspection
            printf(("[Deserializer] [\x1B[1;37mINFO\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_warn_d(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Deserializer] [\x1B[1;33mWARN\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_error_d(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Deserializer] [\x1B[1;31mERROR\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }

    template<typename ...Args>
    void log_info_s(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Serializer] [\x1B[1;37mINFO\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_warn_s(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Serializer] [\x1B[1;33mWARN\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_error_s(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Serializer] [\x1B[1;31mERROR\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }

    template<typename ...Args>
    void log_info(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Main] [\x1B[1;37mINFO\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_warn(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Main] [\x1B[1;33mWARN\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }
    template<typename ...Args>
    void log_error(const std::string& message, Args ...args) {
        if (!silent) {
            printf(("[Main] [\x1B[1;31mERROR\x1B[0m] " + message + "\n").c_str(), args...);
        }
    }

    std::string _intc_base(int value) {
        return "\x1B[38;5;50m" + std::to_string(value) + "\x1B[0m";
    }

    void ensureReasonable(int value, int min = -1000, int max = 10000, int warnMin = 0, int warnMax = 4096) {
        if (value < min) {
            if (unusualNumbers >= 3) {log_error("Aborting due to excessive unusual numbers"); exit(1);}
            log_error("Value is too low: " + _intc_base(value) + " (min: " + _intc_base(min) + ")");
            unusualNumbers++;
        } else if (value > max) {
            if (unusualNumbers >= 3) {log_error("Aborting due to excessive unusual numbers"); exit(1);}
            log_error("Value is too high: " + _intc_base(value) + " (max: " + _intc_base(max) + ")");
            unusualNumbers++;
        } else if (value < warnMin) {
            if (unusualNumbers >= 3) {log_error("Aborting due to excessive unusual numbers"); exit(1);}
            log_warn("Value is unusually low: " + _intc_base(value) + " (min: " + _intc_base(warnMin) + ")");
        } else if (value > warnMax) {
            if (unusualNumbers >= 3) {log_error("Aborting due to excessive unusual numbers"); exit(1);}
            log_warn("Value is unusually high: " + _intc_base(value) + " (max: " + _intc_base(warnMax) + ")");
        }
    }

    std::string add_commas(int value) {
        std::stringstream ss;
        ss << value;
        std::string s = ss.str();
        for (int i = (int)s.length() - 3; i > 0; i -= 3) {
            s = s.insert(i, ",");
        }
        return s;
    }
    std::string intc(int value, int min = -1000, int max = 10000, int warnMin = 0, int warnMax = 4096) {
        // _intc_base with checks and formatting
        ensureReasonable(value, min, max, warnMin, warnMax);
        return "\x1B[38;5;50m" + add_commas(value) + "\x1B[0m";
    }
    std::string floatc(float value) {
        return "\x1B[38;5;219m" + std::to_string(value) + "\x1B[0m";
    }

    std::vector<std::string> splitString(const std::string& string, const std::string& delimiter) {
        std::vector<std::string> result;
        auto start = 0U;
        auto end = string.find(delimiter);
        while (end != std::string::npos) {
            result.push_back(string.substr(start, end - start));
            start = end + delimiter.length();
            end = string.find(delimiter, start);
        }
        result.push_back(string.substr(start, end));
        return result;
    }

    bool directory_of_file_exists(const std::string& fp) {
        std::string dir = fp.substr(0, fp.find_last_of('/'));
        return std::filesystem::exists(dir);
    }

    std::string ticks_to_datetime(long long ticks) {
        if (ticks == 0) return "(never)";
        // convert ticks to seconds
        long unixEpochSeconds = 62135596800; // 1/1/1970
        long ticksPerSecond = 10000000; // 10 million ticks per second
        long seconds = ticks / ticksPerSecond;
        long dateTimeSeconds = seconds - unixEpochSeconds;
        // convert seconds to datetime
        std::time_t time = dateTimeSeconds;
        std::tm* tm = std::gmtime(&time);
        std::stringstream ss;
        ss << std::put_time(tm, "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
}
namespace U = Utils;  // lazy

class Deserializer {
public:
    std::string path;
    std::ifstream file;
    explicit Deserializer(std::string path) {
        this->path = std::move(path);

        std::ifstream _file(this->path, std::ios::binary);
        if (!_file.is_open()) {
            Utils::log_error_d("Failed to open file: " + this->path);
            exit(1);
        }
        this->file = std::move(_file);
    }
    ~Deserializer() {
        this->file.close();
    }
    Layout deserializeLayout() {
        Layout layout;
        // NOTE: This has to be ordered, as it reads the file in the order it is written

        // first, we get the version, which is used to determine which fields are present
        this->getVersion(layout.version, layout.isModded);

        if (layout.isModded) {
            Utils::log_info_d("Using modded layout support");
        }

        Utils::ensureReasonable(layout.version, 0, 100, 0, 50);
        Utils::log_info_d("Deserializing layout version %s", U::intc(layout.version).c_str());
        if (layout.version > MAX_VERSION) {
            Utils::log_warn_d("Layout saved with a newer version of the layout format. This may cause problems.");
        }
        // then we get the stub key, which is the theme of the layout, e.g. "Western"
        layout.stubKey = this->getStubKey();
        Utils::log_info_d("Layout stub key: %s", layout.stubKey.c_str());
        // then we get the name of the layout, e.g. "Western"
        // this is just used for aesthetics
        Utils::log_info_d("Layout theme name: %s", Utils::prettyPrintStubKeyToTheme(layout.stubKey).c_str());

        if (layout.version >= 19) {
            // if the version is 19 or higher, we need to deserialize the anchors
            layout.anchors = this->deserializeAnchors();
        }

        if (layout.version >= 5) {
            // if the version is 5 or higher, we need to deserialize the hydraulic phases
            layout.phases = this->deserializePhases();
        }

        // if the version is greater than 4, we can call the deserializeBridge function.
        if (layout.version > 4) {
            layout.bridge = this->deserializeBridge();
        } else {
            Utils::log_warn_d("Deserializing bridge with version under 5, consider upgrading");
            // otherwise, we have a lot less bridge data to deal with.
            // first, we deserialize the joints.
            int count = this->readInt32();
            Utils::log_info_d("Bridge joint count: %s", U::intc(count).c_str());
            for (int i = 0; i < count; i++) {
                layout.bridge.joints.push_back(this->deserializeJoint());
            }

            // next, the edges.
            count = this->readInt32();
            Utils::log_info_d("Bridge edge count: %s", U::intc(count).c_str());
            for (int i = 0; i < count; i++) {
                layout.bridge.edges.push_back(this->deserializeEdge(layout.bridge.version));
            }

            // last, the pistons.
            count = this->readInt32();
            Utils::log_info_d("Bridge piston count: %s", U::intc(count).c_str());
            for (int i = 0; i < count; i++) {
                layout.bridge.pistons.push_back(this->deserializePiston(layout.bridge.version));
            }
        }

        // After that, if the version is 7 or greater, we can deserialize the Z-axis vehicles (boats, etc.).
        if (layout.version >= 7) {
            layout.zAxisVehicles = this->deserializeZAxisVehicles(layout.version);
        }

        // Then, we can deserialize the vehicles.
        layout.vehicles = this->deserializeVehicles();

        // Next, we deserialize the vehicle stop triggers.
        layout.vehicleStopTriggers = this->deserializeVehicleStopTriggers();

        // If the version is below 20, we deserialize the theme objects, though it's obsolete.
        // This isn't actually collected or used when the layout is loaded in the game, but it's still useful for debugging.
        if (layout.version < 20) {
            layout.themeObjects_OBSOLETE = this->deserializeThemeObjects_OBSOLETE();
        }

        // After that, we can deserialize the event timelines.
        layout.eventTimelines = this->deserializeEventTimelines(layout.version);

        // Then, we deserialize checkpoints.
        layout.checkpoints = this->deserializeCheckpoints();

        // Next, we deserialize terrain stretches.
        layout.terrainStretches = this->deserializeTerrainIslands(layout.version);

        // Deserialize the platforms.
        layout.platforms = this->deserializePlatforms(layout.version);

        // Then, the ramps.
        layout.ramps = this->deserializeRamps(layout.version);

        // Hydraulic phases are here if the layout version is less than 5.
        if (layout.version < 5) {
            layout.phases = this->deserializePhases();
        }

        // Next, we deserialize the vehicle restart phases.
        layout.vehicleRestartPhases = this->deserializeVehicleRestartPhases();

        // Next, flying objects such as airplanes, blimps, etc.
        layout.flyingObjects = this->deserializeFlyingObjects();

        // Then, we deserialize rocks.
        layout.rocks = this->deserializeRocks();

        // After that, we deserialize water blocks.
        layout.waterBlocks = this->deserializeWaterBlocks(layout.version);

        // If the version is less than 5, there's some garbage data here.
        if (layout.version < 5) {
            Utils::log_warn_d("Deserializing garbage data with version under 5");
            int count = this->readInt32();
            int count2;
            for (int i = 0; i < count; i++) {
                this->readString();
                count2 = this->readInt32();
                for (int j = 0; j < count2; j++) {
                    this->readString();
                }
            }
        }

        // Now, we can deserialize the budget.
        layout.budget = this->deserializeBudget();

        // Then, the settings.
        layout.settings = this->deserializeSettings();

        // Now, if the version is 9 or above, we have custom shapes to deal with.
        if (layout.version >= 9) {
            layout.customShapes = this->deserializeCustomShapes(layout.version);
        }

        // Deserialize workshop binary (version 15+)
        if (layout.version >= 15) {
            layout.workshop = this->deserializeWorkshop(layout.version);
        }

        // Deserialize support pillars (version 17+)
        if (layout.version >= 17) {
            layout.supportPillars = this->deserializeSupportPillars();
        }

        // Finally, we can deserialize pillars. (version 18+)
        if (layout.version >= 18) {
            layout.pillars = this->deserializePillars();
        }

        // Now, some checks:
        // First, we check to make sure we are at the end of the file.
        long pos = (long)this->file.tellg();
        // Modded layouts have extra mod data at the end.
        if (!layout.isModded) return layout;  // We can end here if there's no mod data.

        // MOD SUPPORT: PTF stores mod data at the end, indicated by a negative version at the start.
        // Notify user that we're deserializing mod data.
        Utils::log_info_d("Deserializing mod data...");
        // Then, we deserialize the mod data.
        layout.modData = this->deserializePTFModData();
        // Then, we can return the layout.
        return layout;
    }
    static float fixPistonNormalizedValue(float value) {
        float out;
        if (value < 0.25f) {
            out = lerp(1.0f, 0.5f, clamp01(value / 0.25f));
            return out;
        }
        if (value > 0.75f) {
            out = lerp(0.5f, 1.0f, clamp01((value - 0.75f) / 0.25f));
            return out;
        }
        out = lerp(0.0f, 0.5f, clamp01(std::abs(value - 0.5f) / 0.25f));
        return out;
    }
private:
    char* readByte() {
        char* byte = new char[1];
        this->file.read(byte, 1);
        return byte;
    }
    char* readBytes(int count) {
        char* bytes = new char[count];
        this->file.read(bytes, count);
        return bytes;
    }
    int8_t readInt8() {
        int8_t value;
        this->file.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    }
    bool readBool() {
        return this->readInt8() != 0;
    }
    int16_t readInt16() {
        int16_t value;
        this->file.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    }
    uint16_t readUInt16() {
        uint16_t value;
        this->file.read(reinterpret_cast<char*>(&value), sizeof(value));
        return value;
    }
    int32_t readInt32() {
        int32_t value;
        this->file.read(reinterpret_cast<char*>(&value), sizeof(int32_t));
        return value;
    }
    float readFloat() {
        float value;
        this->file.read(reinterpret_cast<char*>(&value), 4);
        return value;
    }
    std::string readString() {
        int length = this->readUInt16();
        char* bytes = this->readBytes(length);
        std::string str(bytes, length);
        delete[] bytes;
        return str;
    }
    char** readByteArray() {
        int length = this->readInt32();
        if (length > 0) {
            char** array = new char*[length];
            // equivalent of Buffer.BlockCopy in .NET
            for (int i = 0; i < length; i++) {
                array[i] = this->readBytes(1);
            }
            return array;
        } else {
            Utils::log_error_d("Failed to read byte array: length is less than or equal to 0");
            exit(1);
        }
    }
    void getVersion(int &version, bool &isModded) {
        // The version is used for the order things are parsed in.
        version = this->readInt32();
        isModded = false;

        if (version < 0) {
            // PolyTechFramework multiplies the version by -1 to make it opposite of the original, so we need to reverse that (and mark the layout as modded).
            version = -version;
            isModded = true;
        }
    }
    std::string getStubKey() {
        // The stub key is the theme of the layout, e.g. "Western"
        return this->readString();
    }
    Vec3 readVec3() {
        Vec3 vec{};
        vec.x = this->readFloat();
        vec.y = this->readFloat();
        vec.z = this->readFloat();
        return vec;
    }
    Vec2 readVec2() {
        Vec2 vec{};
        vec.x = this->readFloat();
        vec.y = this->readFloat();
        return vec;
    }
    Color readColor() {
        // Colors are a bit weird; r, g, and b are stored in a single byte.
        // There is no alpha channel.
        Color color{};
        color.r = (float)reinterpret_cast<uint8_t*>(this->readByte())[0] / 255.0f;
        color.g = (float)reinterpret_cast<uint8_t*>(this->readByte())[0] / 255.0f;
        color.b = (float)reinterpret_cast<uint8_t*>(this->readByte())[0] / 255.0f;
        color.a = 1.0f;
        return color;
    }
    Quaternion readQuaternion() {
        Quaternion quat{};
        quat.x = this->readFloat();
        quat.y = this->readFloat();
        quat.z = this->readFloat();
        quat.w = this->readFloat();
        return quat;
    }
    BridgeJoint deserializeAnchor() {
        BridgeJoint anchor{};
        anchor.pos = this->readVec3();
        anchor.is_anchor = this->readBool();
        anchor.is_split = this->readBool();
        anchor.guid = this->readString();
        return anchor;
    }
    std::vector<BridgeJoint> deserializeAnchors() {
        std::vector<BridgeJoint> anchors;
        int count = this->readInt32();
        Utils::log_info_d("Anchor count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            anchors.push_back(this->deserializeAnchor());
        }
        return anchors;
    }
    HydraulicPhase deserializePhase() {
        HydraulicPhase phase{};
        phase.time_delay = this->readFloat();
        phase.guid = this->readString();
        return phase;
    }
    std::vector<HydraulicPhase> deserializePhases() {
        int count = this->readInt32();
        Utils::log_info_d("HydraulicPhase count: %s", U::intc(count).c_str());
        std::vector<HydraulicPhase> phases;
        for (int i = 0; i < count; i++) {
            phases.push_back(this->deserializePhase());
        }
        return phases;
    }
    BridgeJoint deserializeJoint() {
        BridgeJoint joint{};
        joint.pos = this->readVec3();
        joint.is_anchor = this->readBool();
        joint.is_split = this->readBool();
        joint.guid = this->readString();
        return joint;
    }
    BridgeEdge deserializeEdge(int version) {
        BridgeEdge edge{};
        edge.material_type = (BridgeMaterialType)this->readInt32();
        edge.node_a_guid = this->readString();
        edge.node_b_guid = this->readString();
        edge.joint_a_part = (SplitJointPart)this->readInt32();
        edge.joint_b_part = (SplitJointPart)this->readInt32();
        edge.guid = (version >= 11) ? this->readString() : "";
        return edge;
    }
    BridgeSpring deserializeSpring() {
        BridgeSpring spring{};
        spring.normalized_value = this->readFloat();
        spring.node_a_guid = this->readString();
        spring.node_b_guid = this->readString();
        spring.guid = this->readString();
        return spring;
    }
    static float clamp01(float value) {
        if (value < 0.0f) {
            return 0.0f;
        } else if (value > 1.0f) {
            return 1.0f;
        } else {
            return value;
        }
    }
    static float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }
    Piston deserializePiston(int version) {
        Piston piston{};
        piston.normalized_value = this->readFloat();
        piston.node_a_guid = this->readString();
        piston.node_b_guid = this->readString();
        piston.guid = this->readString();

        // Fix the normalized value if the version is less than 8.
        if (version < 8) {
            piston.normalized_value = fixPistonNormalizedValue(piston.normalized_value);
        }
        return piston;
    }
    BridgeSplitJoint deserializeSplitJoint() {
        BridgeSplitJoint split_joint{};
        split_joint.guid = this->readString();
        split_joint.state = (SplitJointState)this->readInt32();
        return split_joint;
    }
    HydraulicsControllerPhase deserializeHydraulicControllerPhase(int version) {
        HydraulicsControllerPhase phase{};
        phase.hydraulics_phase_guid = this->readString();

        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            phase.piston_guids.push_back(this->readString());
        }

        if (version > 2) {
            count = this->readInt32();
            for (int i = 0; i < count; i++) {
                phase.bridge_split_joints.push_back(this->deserializeSplitJoint());
            }
        } else {
            count = this->readInt32();
            for (int i = 0; i < count; i++) {
                // garbage data
                this->readString();
            }
        }
        if (version > 9) {
            phase.disable_new_additions = this->readBool();
        }
        return phase;
    }
    Bridge deserializeBridge() {
        Utils::log_info_d("Deserializing bridge...");
        // Bridges are a pretty large structure, and there's a lot of nested fields.

        Bridge bridge{};
        // First, we read the version of the bridge.
        bridge.version = this->readInt32();
        Utils::ensureReasonable(bridge.version, 0, 100, 0, 50);
        Utils::log_info_d("Bridge version: %s", U::intc(bridge.version).c_str());
        if (bridge.version > MAX_BRIDGE_VERSION) {
            Utils::log_warn_d("Bridge saved with a newer version of the bridge format. This may cause problems.");
        }

        // If the version is less than 2, we don't have any of the following fields.
        if (bridge.version < 2) {
            Utils::log_warn_d("Bridge version is less than 2, skipping bridge deserialization.");
            return bridge;
        }

        // Next, we read the number of joints, and deserialize them.
        int count = this->readInt32();
        Utils::log_info_d("Bridge joint count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            bridge.joints.push_back(this->deserializeJoint());
        }

        // Then, we read the number of edges, and deserialize them.
        count = this->readInt32();
        Utils::log_info_d("Bridge edge count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            bridge.edges.push_back(this->deserializeEdge(bridge.version));
        }

        // If the version is 7 or above, we read the bridge's springs.
        if (bridge.version >= 7) {
            count = this->readInt32();
            Utils::log_info_d("Bridge spring count: %s", U::intc(count).c_str());
            for (int i = 0; i < count; i++) {
                bridge.springs.push_back(this->deserializeSpring());
            }
        }

        // After that, we can read the number of pistons and deserialize them.
        count = this->readInt32();
        Utils::log_info_d("Bridge piston count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            bridge.pistons.push_back(this->deserializePiston(bridge.version));
        }

        // Then, we read the hydraulic phases.
        count = this->readInt32();
        Utils::log_info_d("Bridge hydraulic phase count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            bridge.phases.push_back(this->deserializeHydraulicControllerPhase(bridge.version));
        }

        // if the version is 5, there's some garbage data.
        if (bridge.version == 5) {
            Utils::log_warn_d("Discarding v5 garbage data.");
            count = this->readInt32();
            for (int i = 0; i < count; i++) {
                this->readString();
            }
        }

        // if the version is 6 or above, we read the anchors.
        if (bridge.version >= 6) {
            count = this->readInt32();
            Utils::log_info_d("Bridge anchor count: %s", U::intc(count).c_str());
            for (int i = 0; i < count; i++) {
                bridge.anchors.push_back(this->deserializeAnchor());
            }
        }

        // finally if the version is 4 or above and less than 9, there's a random bool at the end.
        if (bridge.version >= 4 && bridge.version < 9) {
            Utils::log_warn_d("Discarding v4-8 garbage data.");
            this->readBool();
        }

        Utils::log_info_d("Bridge deserialization complete.");
        return bridge;
    }
    ZAxisVehicle deserializeZAxisVehicle(int version) {
        ZAxisVehicle vehicle{};
        vehicle.pos = this->readVec2();
        vehicle.prefab_name = this->readString();
        vehicle.guid = this->readString();
        vehicle.time_delay = this->readFloat();
        // If the version is 8 or above, we read the vehicle's speed.
        if (version >= 8) {
            vehicle.speed = this->readFloat();
        }
        // If the version is 26 or above, we read the vehicle's rotation.
        if (version >= 26) {
            vehicle.rot = this->readQuaternion();
            vehicle.rotation_degrees = this->readFloat();
        }

        return vehicle;
    }
    std::vector<ZAxisVehicle> deserializeZAxisVehicles(int version) {
        int count = this->readInt32();
        Utils::log_info_d("ZedAxisVehicle count: %s", U::intc(count).c_str());
        std::vector<ZAxisVehicle> vehicles;
        for (int i = 0; i < count; i++) {
            vehicles.push_back(this->deserializeZAxisVehicle(version));
        }
        return vehicles;
    }
    Vehicle deserializeVehicle() {
        Vehicle vehicle{};
        vehicle.display_name = this->readString();
        vehicle.pos = this->readVec2();
        vehicle.rot = this->readQuaternion();
        vehicle.prefab_name = this->readString();
        vehicle.target_speed = this->readFloat();
        vehicle.mass = this->readFloat();
        vehicle.braking_force_multiplier = this->readFloat();
        vehicle.strength_method = (StrengthMethod)this->readInt32();
        vehicle.acceleration = this->readFloat();
        vehicle.max_slope = this->readFloat();
        vehicle.desired_acceleration = this->readFloat();
        vehicle.shocks_multiplier = this->readFloat();
        vehicle.rotation_degrees = this->readFloat();
        vehicle.time_delay = this->readFloat();
        vehicle.idle_on_downhill = this->readBool();
        vehicle.flipped = this->readBool();
        vehicle.ordered_checkpoints = this->readBool();
        vehicle.guid = this->readString();

        // Deserialize the checkpoint GUIDs.
        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            vehicle.checkpoint_guids.push_back(this->readString());
        }

        return vehicle;
    }
    std::vector<Vehicle> deserializeVehicles() {
        int count = this->readInt32();
        Utils::log_info_d("Vehicle count: %s", U::intc(count).c_str());
        std::vector<Vehicle> vehicles;
        for (int i = 0; i < count; i++) {
            vehicles.push_back(this->deserializeVehicle());
        }
        return vehicles;
    }
    VehicleStopTrigger deserializeVehicleStopTrigger() {
        VehicleStopTrigger trigger{};
        trigger.pos = this->readVec2();
        trigger.rot = this->readQuaternion();
        trigger.height = this->readFloat();
        trigger.rotation_degrees = this->readFloat();
        trigger.flipped = this->readBool();
        trigger.prefab_name = this->readString();
        trigger.stop_vehicle_guid = this->readString();
        return trigger;
    }
    std::vector<VehicleStopTrigger> deserializeVehicleStopTriggers() {
        int count = this->readInt32();
        Utils::log_info_d("VehicleStopTrigger count: %s", U::intc(count).c_str());
        std::vector<VehicleStopTrigger> triggers;
        for (int i = 0; i < count; i++) {
            triggers.push_back(this->deserializeVehicleStopTrigger());
        }
        return triggers;
    }
    ThemeObject deserializeThemeObject_OBSOLETE() {
        ThemeObject object{};
        object.pos = this->readVec2();
        object.prefab_name = this->readString();
        object.unknown_value = this->readBool();
        return object;
    }
    std::vector<ThemeObject> deserializeThemeObjects_OBSOLETE() {
        int count = this->readInt32();
        Utils::log_warn_d("ThemeObjects are obsolete, consider upgrading the layout version.");
        Utils::log_info_d("ThemeObject count: %s", U::intc(count).c_str());
        std::vector<ThemeObject> objects;
        for (int i = 0; i < count; i++) {
            objects.push_back(this->deserializeThemeObject_OBSOLETE());
        }
        return objects;
    }
    EventUnit deserializeEventUnit(int version) {
        EventUnit unit{};
        // kind of looks like somebody messed up version compatibility and only realized after 3 versions that this was an issue
        if (version >= 7) {
            unit.guid = this->readString();
            return unit;
        }

        // what is the point of this
        std::string text = this->readString();
        if (!text.empty()) {
            unit.guid = text;
        }

        text = this->readString();
        if (!text.empty()) {
            unit.guid = text;
        }

        text = this->readString();
        if (!text.empty()) {
            unit.guid = text;
        }

        return unit;
    }
    EventStage deserializeEventStage(int version) {
        EventStage stage{};
        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            stage.units.push_back(this->deserializeEventUnit(version));
        }
        return stage;
    }
    EventTimeline deserializeEventTimeline(int version) {
        EventTimeline timeline{};
        // forgot this field originally, memory usage go brrrrr
        timeline.checkpoint_guid = this->readString();
        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            timeline.stages.push_back(this->deserializeEventStage(version));
        }
        return timeline;
    }
    std::vector<EventTimeline> deserializeEventTimelines(int version) {
        int count = this->readInt32();
        Utils::log_info_d("EventTimeline count: %s", U::intc(count).c_str());
        std::vector<EventTimeline> timelines;
        for (int i = 0; i < count; i++) {
            timelines.push_back(this->deserializeEventTimeline(version));
        }
        return timelines;
    }
    Checkpoint deserializeCheckpoint() {
        Checkpoint checkpoint{};
        checkpoint.pos = this->readVec2();
        checkpoint.prefab_name = this->readString();
        checkpoint.vehicle_guid = this->readString();
        checkpoint.vehicle_restart_phase_guid = this->readString();
        checkpoint.trigger_timeline = this->readBool();
        checkpoint.stop_vehicle = this->readBool();
        checkpoint.reverse_vehicle_on_restart = this->readBool();
        checkpoint.guid = this->readString();
        return checkpoint;
    }
    std::vector<Checkpoint> deserializeCheckpoints() {
        int count = this->readInt32();
        Utils::log_info_d("Checkpoint count: %s", U::intc(count).c_str());
        std::vector<Checkpoint> checkpoints;
        for (int i = 0; i < count; i++) {
            checkpoints.push_back(this->deserializeCheckpoint());
        }
        return checkpoints;
    }
    Platform deserializePlatform(int version) {
        Platform platform{};
        platform.pos = this->readVec2();
        platform.width = this->readFloat();
        platform.height = this->readFloat();
        platform.flipped = this->readBool();
        // in version 22 and above, there's a field of whether the platform is solid or not
        if (version >= 22) {
            platform.solid = this->readBool();
            return platform;
        }
        // random int at the end of earlier versions
        this->readInt32();
        return platform;
    }
    std::vector<Platform> deserializePlatforms(int version) {
        int count = this->readInt32();
        Utils::log_info_d("Platform count: %s", U::intc(count).c_str());
        std::vector<Platform> platforms;
        for (int i = 0; i < count; i++) {
            platforms.push_back(this->deserializePlatform(version));
        }
        return platforms;
    }
    TerrainIsland deserializeTerrainStretch(int version) {
        TerrainIsland island{};
        island.pos = this->readVec3();
        island.prefab_name = this->readString();
        island.height_added = this->readFloat();
        island.right_edge_water_height = this->readFloat();
        island.terrain_island_type = (TerrainIslandType)this->readInt32();
        island.variant_index = this->readInt32();
        island.flipped = this->readBool();
        if (version >= 6) {
            island.lock_position = this->readBool();
        }
        return island;
    }
    std::vector<TerrainIsland> deserializeTerrainIslands(int version) {
        int count = this->readInt32();
        Utils::log_info_d("TerrainIsland count: %s", U::intc(count).c_str());
        std::vector<TerrainIsland> islands;
        for (int i = 0; i < count; i++) {
            islands.push_back(this->deserializeTerrainStretch(version));
        }
        return islands;
    }
    Ramp deserializeRamp(int version) {
        Ramp ramp{};
        ramp.pos = this->readVec2();
        // deserialize control points
        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            ramp.control_points.push_back(this->readVec2());
        }

        ramp.height = std::abs(this->readFloat());
        ramp.num_segments = this->readInt32();
        ramp.spline_type = (SplineType)this->readInt32();
        ramp.flipped_vertical = this->readBool();
        ramp.flipped_horizontal = this->readBool();
        ramp.hide_legs = (version >= 23 && this->readBool());

        if (version >= 25) {
            ramp.flipped_legs = this->readBool();
        } else if (version >= 22) {
            this->readBool();
        } else {
            this->readInt32();
        }

        if (version >= 13) {
            count = this->readInt32();
            for (int i = 0; i < count; i++) {
                ramp.line_points.push_back(this->readVec2());
            }
        }

        return ramp;
    }
    std::vector<Ramp> deserializeRamps(int version) {
        int count = this->readInt32();
        Utils::log_info_d("Ramp count: %s", U::intc(count).c_str());
        std::vector<Ramp> ramps;
        for (int i = 0; i < count; i++) {
            ramps.push_back(this->deserializeRamp(version));
        }
        return ramps;
    }
    VehicleRestartPhase deserializeVehicleRestartPhase() {
        VehicleRestartPhase phase{};
        phase.time_delay = this->readFloat();
        phase.guid = this->readString();
        phase.vehicle_guid = this->readString();
        return phase;
    }
    std::vector<VehicleRestartPhase> deserializeVehicleRestartPhases() {
        int count = this->readInt32();
        Utils::log_info_d("VehicleRestartPhase count: %s", U::intc(count).c_str());
        std::vector<VehicleRestartPhase> phases;
        for (int i = 0; i < count; i++) {
            phases.push_back(this->deserializeVehicleRestartPhase());
        }
        return phases;
    }
    FlyingObject deserializeFlyingObject() {
        FlyingObject object;
        object.pos = this->readVec3();
        object.scale = this->readVec3();
        object.prefab_name = this->readString();
        return object;
    }
    std::vector<FlyingObject> deserializeFlyingObjects() {
        int count = this->readInt32();
        Utils::log_info_d("FlyingObject count: %s", U::intc(count).c_str());
        std::vector<FlyingObject> objects;
        for (int i = 0; i < count; i++) {
            objects.push_back(this->deserializeFlyingObject());
        }
        return objects;
    }
    Rock deserializeRock() {
        Rock rock{};
        rock.pos = this->readVec3();
        rock.scale = this->readVec3();
        rock.prefab_name = this->readString();
        rock.flipped = this->readBool();
        return rock;
    }
    std::vector<Rock> deserializeRocks() {
        int count = this->readInt32();
        Utils::log_info_d("Rock count: %s", U::intc(count).c_str());
        std::vector<Rock> rocks;
        for (int i = 0; i < count; i++) {
            rocks.push_back(this->deserializeRock());
        }
        return rocks;
    }
    WaterBlock deserializeWaterBlock(int version) {
        WaterBlock block{};
        block.pos = this->readVec3();
        block.width = this->readFloat();
        block.height = this->readFloat();
        if (version >= 12) {
            block.lock_position = this->readBool();
        }
        return block;
    }
    std::vector<WaterBlock> deserializeWaterBlocks(int version) {
        int count = this->readInt32();
        Utils::log_info_d("WaterBlock count: %s", U::intc(count).c_str());
        std::vector<WaterBlock> blocks;
        for (int i = 0; i < count; i++) {
            blocks.push_back(this->deserializeWaterBlock(version));
        }
        return blocks;
    }
    Budget deserializeBudget() {
        Budget b{};
        b.cash = this->readInt32();
        Utils::log_info_d("Budget: $%s", U::intc(b.cash, 0, 100000000, 0, 100000000).c_str());
        b.road = this->readInt32();
        b.wood = this->readInt32();
        b.steel = this->readInt32();
        b.hydraulics = this->readInt32();
        b.rope = this->readInt32();
        b.cable = this->readInt32();
        b.spring = this->readInt32();
        b.bungee_rope = this->readInt32();
        b.allow_wood = this->readBool();
        b.allow_steel = this->readBool();
        b.allow_hydraulics = this->readBool();
        b.allow_rope = this->readBool();
        b.allow_cable = this->readBool();
        b.allow_spring = this->readBool();
        b.allow_reinforced_road = this->readBool();
        return b;
    }
    Settings deserializeSettings() {
        Settings settings{};
        settings.hydraulics_controller_enabled = this->readBool();
        Utils::log_info_d("Hydraulics controller: %s", settings.hydraulics_controller_enabled ? "\x1B[1;92menabled\x1B[0m" : "\x1B[1;91mdisabled\x1B[0m");
        settings.unbreakable = this->readBool();
        Utils::log_info_d("Unbreakable mode: %s", settings.unbreakable ? "\x1B[1;92menabled\x1B[0m" : "\x1B[1;91mdisabled\x1B[0m");
        return settings;
    }
    CustomShape deserializeCustomShape(int version) {
        CustomShape s{};
        s.pos = this->readVec3();
        s.rot = this->readQuaternion();
        s.scale = this->readVec3();
        s.flipped = this->readBool();
        s.dynamic = this->readBool();
        s.collides_with_road = this->readBool();
        s.collides_with_nodes = this->readBool();
        if (version >= 25) {
            s.collides_with_split_nodes = this->readBool();
        }
        s.rotation_degrees = this->readFloat();
        if (version >= 10) {
            s.color = this->readColor();
        } else {
            readInt32();
        }
        if (version >= 11) {
            s.mass = this->readFloat();
        } else {
            this->readFloat();
            s.mass = 40.0f;
        }
        if (version >= 14) {
            s.bounciness = this->readFloat();
        } else {
            s.bounciness = 0.5f;
        }
        if (version >= 24) {
            s.pin_motor_strength = this->readFloat();
            s.pin_target_velocity = this->readFloat();
        } else {
            s.pin_motor_strength = 0.0f;
            s.pin_target_velocity = 0.0f;
        }

        // Deserialize points binary
        int count = this->readInt32();
        for (int i = 0; i < count; i++) {
            s.points_local_space.push_back(this->readVec2());
        }

        // Deserialize static pins binary
        count = this->readInt32();
        for (int i = 0; i < count; i++) {
            Vec3 pos = this->readVec3();
            pos.z = -1.348f;
            s.static_pins.push_back(pos);
        }

        // Deserialize dynamic anchors binary
        count = this->readInt32();
        for (int i = 0; i < count; i++) {
            s.dynamic_anchor_guids.push_back(this->readString());
        }

        return s;
    }
    std::vector<CustomShape> deserializeCustomShapes(int version) {
        int count = this->readInt32();
        Utils::log_info_d("Custom shape count: %s", U::intc(count).c_str());
        std::vector<CustomShape> shapes;
        for (int i = 0; i < count; i++) {
            shapes.push_back(this->deserializeCustomShape(version));
        }
        return shapes;
    }
    Workshop deserializeWorkshop(int version) {
        Workshop workshop{};
        workshop.id = this->readString();
        Utils::log_info_d("Workshop ID: \x1B[1;95m" + workshop.id + "\x1B[0m");
        if (version >= 16) {
            workshop.leaderboard_id = this->readString();
            Utils::log_info_d("Workshop leaderboard ID: \x1B[1;95m" + workshop.leaderboard_id + "\x1B[0m");
        }
        workshop.title = this->readString();
        Utils::log_info_d("Workshop title: \x1B[1;95m" + workshop.title + "\x1B[0m");
        workshop.description = this->readString();
        Utils::log_info_d("Workshop description: \x1B[1;95m\n" + workshop.description + "\x1B[0m");
        workshop.autoplay = this->readBool();
        Utils::log_info_d("Autoplay: %s", workshop.autoplay ? "\x1B[1;92yes\x1B[0m" : "\x1B[1;91mno\x1B[0m");
        int count = this->readInt32();
        Utils::log_info_d("Tag count: %s", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            workshop.tags.push_back(this->readString());
        }
        return workshop;
    }
    SupportPillar deserializeSupportPillar() {
        SupportPillar pillar{};
        pillar.pos = this->readVec3();
        pillar.scale = this->readVec3();
        pillar.prefab_name = this->readString();
        return pillar;
    }
    std::vector<SupportPillar> deserializeSupportPillars() {
        int count = this->readInt32();
        Utils::log_info_d("SupportPillar count: %s", U::intc(count).c_str());
        std::vector<SupportPillar> pillars;
        for (int i = 0; i < count; i++) {
            pillars.push_back(this->deserializeSupportPillar());
        }
        return pillars;
    }
    Pillar deserializePillar() {
        Pillar pillar{};
        pillar.pos = this->readVec3();
        pillar.height = this->readFloat();
        pillar.prefab_name = this->readString();
        return pillar;
    }
    std::vector<Pillar> deserializePillars() {
        int count = this->readInt32();
        Utils::log_info_d("Pillars count: %s", U::intc(count).c_str());
        std::vector<Pillar> pillars;
        for (int i = 0; i < count; i++) {
            pillars.push_back(this->deserializePillar());
        }
        return pillars;
    }
    // MOD SUPPORT
    ModData deserializePTFModData() {
        ModData mod_data{};

        int count = this->readInt16();
        Utils::log_info_d("Layout saved with %s mods", U::intc(count).c_str());
        for (int i = 0; i < count; i++) {
            std::string string = this->readString();
            std::vector<std::string> partsOfMod = Utils::splitString(string, "\u058D");
            std::string name = !partsOfMod.empty() ? partsOfMod[0] : "";
            std::string version = partsOfMod.size() >= 2 ? partsOfMod[1] : "";
            std::string settings = partsOfMod.size() >= 3 ? partsOfMod[2] : "";

            Utils::log_info_d("Name: \x1B[1;95m" + name + "\x1B[0m");
            Utils::log_info_d("Version: \x1B[1;95m" + version + "\x1B[0m");
            Utils::log_info_d("Settings: \x1B[1;95m" + settings + "\x1B[0m\n");

            mod_data.mods.push_back(Mod{name, version, settings});
        }

        // check if that's everything
        long pos = (long) this->file.tellg();
        if (pos == this->file.seekg(0, std::ios::end).tellg()) {
            return mod_data;
        }
        this->file.seekg(pos);

        // if not, read the save data
        int extraSaveDataCount = this->readInt32();
        if (extraSaveDataCount == 0) return mod_data;
        Utils::log_info_d("Mod save data count: %s", U::intc(extraSaveDataCount).c_str());

        for (int i = 0; i < extraSaveDataCount; i++) {
            std::string modIdentifier = this->readString();

            std::vector<std::string> partsOfMod = Utils::splitString(modIdentifier, "\u058D");
            std::string name = !partsOfMod.empty() ? partsOfMod[0] : "";
            std::string version = partsOfMod.size() >= 2 ? partsOfMod[1] : "";

            // if the name is empty, the mod is invalid
            if (name.empty()) {
                Utils::log_warn_d("Invalid mod identifier: \x1B[1;95m" + modIdentifier + "\x1B[0m");
                continue;
            }

            Utils::log_info_d("Name: \x1B[1;95m" + name + "\x1B[0m");
            Utils::log_info_d("Version: \x1B[1;95m" + version + "\x1B[0m");

            char *customModSaveData = reinterpret_cast<char *>(this->readByteArray());

            mod_data.mod_save_data.push_back(ModSaveData{customModSaveData, name, version});
        }
        return mod_data;
    }
};

class Serializer {
public:
    std::string path;
    std::ofstream file;
    Layout layout;
    explicit Serializer(const std::string &filename, const Layout &layout) {
        this->file = std::ofstream(filename, std::ios::binary | std::ios::out);
        this->layout = layout;
        this->path = filename;

        if (!this->file.is_open()) {
            Utils::log_error_s("Failed to open file for writing: %s", filename.c_str());
            exit(1);
        }
    }
    ~Serializer() {
        this->file.close();
    }
    void serializeLayout() {
        this->serializePreBridgeBinary();
        this->serializeBridgeBinary();
        this->serializePostBridgeBinary();
    }
private:
    void writeInt32(int32_t value) {
        this->file.write(reinterpret_cast<char *>(&value), sizeof(int32_t));
    }
    void writeString(const std::string &value) {
        this->writeUInt16((short)value.length());
        this->file.write(value.c_str(), (long)value.length());
    }
    void writeFloat(float value) {
        this->file.write(reinterpret_cast<char *>(&value), sizeof(float));
    }
    void writeBool(bool value) {
        this->file.write(reinterpret_cast<char *>(&value), sizeof(bool));
    }
    void writeUInt16(uint16_t value) {
        this->file.write(reinterpret_cast<char *>(&value), sizeof(uint16_t));
    }
    void writeByte(uint8_t value) {
        this->file.write(reinterpret_cast<char *>(&value), sizeof(uint8_t));
    }
    void writeVector3(const Vec3 &value) {
        this->writeFloat(value.x);
        this->writeFloat(value.y);
        this->writeFloat(value.z);
    }
    void writeVector2(const Vec2 &value) {
        this->writeFloat(value.x);
        this->writeFloat(value.y);
    }
    void writeColor(const Color &value) {
        this->writeByte((uint8_t)(value.r * 255.0f));
        this->writeByte((uint8_t)(value.g * 255.0f));
        this->writeByte((uint8_t)(value.b * 255.0f));
    }
    void writeQuaternion(const Quaternion &value) {
        this->writeFloat(value.x);
        this->writeFloat(value.y);
        this->writeFloat(value.z);
        this->writeFloat(value.w);
    }

    Vehicle findVehicleByGuid(const std::string& guid) {
        for (auto &vehicle : this->layout.vehicles) {
            if (vehicle.guid == guid) {
                U::log_info_s("Found vehicle '%s' by GUID %s", vehicle.prefab_name.c_str(), guid.c_str());
                return vehicle;
            }
        }
        Utils::log_error_s("Could not find vehicle with GUID \x1B[1;95m" + guid + "\x1B[0m");
        exit(1);
    }

    void serializeAnchorsBinary() {
        this->writeInt32((int)this->layout.anchors.size());
        for (BridgeJoint &anchor : this->layout.anchors) {
            this->writeVector3(anchor.pos);
            this->writeBool(anchor.is_anchor);
            this->writeBool(anchor.is_split);
            this->writeString(anchor.guid);
        }
        U::log_info_s("Serialized %s anchors", U::intc((int)this->layout.anchors.size()).c_str());
    }
    void serializeHydraulicsPhasesBinary() {
        this->writeInt32((int)this->layout.phases.size());
        for (HydraulicPhase &phase : this->layout.phases) {
            this->writeFloat(phase.time_delay);
            this->writeString(phase.guid);
        }
        U::log_info_s("Serialized %s hydraulic phases", U::intc((int)this->layout.phases.size()).c_str());
    }
    void serializePreBridgeBinary() {
        this->writeInt32(MAX_VERSION);
        U::log_info_s("Wrote version %s", U::intc(MAX_VERSION).c_str());
        this->writeString(this->layout.stubKey);
        U::log_info_s("Wrote stub key '%s'", this->layout.stubKey.c_str());
        this->serializeAnchorsBinary();
        this->serializeHydraulicsPhasesBinary();
        this->file << std::flush;
    }
    void serializeBridgeBinary() {
        auto bridge = this->layout.bridge;
        this->writeInt32(MAX_BRIDGE_VERSION); // Version
        U::log_info_s("Serializing bridge version %s", U::intc(MAX_BRIDGE_VERSION).c_str());

        this->writeInt32((int)bridge.joints.size()); // Joint count
        for (const BridgeJoint &joint : bridge.joints) {
            this->writeVector3(joint.pos); // Position
            this->writeBool(joint.is_anchor); // Is anchor
            this->writeBool(joint.is_split); // Is split
            this->writeString(joint.guid); // GUID
        }
        U::log_info_s("Serialized %s joints", U::intc((int)bridge.joints.size()).c_str());

        this->writeInt32((int)bridge.edges.size()); // Edge count
        for (const BridgeEdge &edge : bridge.edges) {
            this->writeInt32(edge.material_type); // Material type
            this->writeString(edge.node_a_guid); // Node A GUID
            this->writeString(edge.node_b_guid); // Node B GUID
            this->writeInt32(edge.joint_a_part); // Joint A part
            this->writeInt32(edge.joint_b_part); // Joint B part
        }
        U::log_info_s("Serialized %s edges", U::intc((int)bridge.edges.size()).c_str());

        this->writeInt32((int)bridge.springs.size()); // Spring count
        for (const BridgeSpring &spring : bridge.springs) {
            this->writeFloat(spring.normalized_value); // Normalized value
            this->writeString(spring.node_a_guid); // Node A GUID
            this->writeString(spring.node_b_guid); // Node B GUID
            this->writeString(spring.guid); // GUID
        }
        U::log_info_s("Serialized %s springs", U::intc((int)bridge.springs.size()).c_str());

        this->writeInt32((int)bridge.pistons.size()); // Piston count
        for (const Piston &piston : bridge.pistons) {
            this->writeFloat(piston.normalized_value); // Normalized value
            this->writeString(piston.node_a_guid); // Node A GUID
            this->writeString(piston.node_b_guid); // Node B GUID
            this->writeString(piston.guid); // GUID
        }
        U::log_info_s("Serialized %s pistons", U::intc((int)bridge.pistons.size()).c_str());

        // Hydraulics controller binary
        this->writeInt32((int)bridge.phases.size()); // Hydraulics phase count
        for (const HydraulicsControllerPhase &phase : bridge.phases) {
            this->writeString(phase.hydraulics_phase_guid); // Hydraulics phase GUID

            this->writeInt32((int)phase.piston_guids.size()); // Piston GUID count
            for (const std::string &piston_guid : phase.piston_guids) {
                this->writeString(piston_guid); // Piston GUID
            }

            this->writeInt32((int)phase.bridge_split_joints.size()); // Bridge split joint count
            for (const BridgeSplitJoint &bridge_split_joint : phase.bridge_split_joints) {
                this->writeString(bridge_split_joint.guid); // Bridge split joint GUID
                this->writeInt32(bridge_split_joint.state); // Bridge split joint state
            }
            this->writeBool(phase.disable_new_additions);
        }
        U::log_info_s("Serialized %s hydraulic phases", U::intc((int)bridge.phases.size()).c_str());

        this->writeInt32((int)bridge.anchors.size()); // Anchor count
        for (const BridgeJoint &anchor : bridge.anchors) {
            this->writeVector3(anchor.pos); // Position
            this->writeBool(anchor.is_anchor); // Is anchor
            this->writeBool(anchor.is_split); // Is split
            this->writeString(anchor.guid); // GUID
        }
        U::log_info_s("Serialized %s anchors", U::intc((int)bridge.anchors.size()).c_str());

        this->file << std::flush;
    }
    void serializePostBridgeBinary() {
        // Z Axis Vehicles
        this->writeInt32((int)this->layout.zAxisVehicles.size()); // Z-Axis vehicle count
        for (const ZAxisVehicle &vehicle : layout.zAxisVehicles) {
            this->writeVector2(vehicle.pos); // Position
            this->writeString(vehicle.prefab_name); // Prefab name
            this->writeString(vehicle.guid); // GUID
            this->writeFloat(vehicle.time_delay); // Time delay (seconds)
            this->writeFloat(vehicle.speed); // Speed
            this->writeQuaternion(vehicle.rot); // Rotation
            this->writeFloat(vehicle.rotation_degrees); // Rotation degrees
        }
        U::log_info_s("Serialized %s z-axis vehicles", U::intc((int)this->layout.zAxisVehicles.size()).c_str());

        // Vehicles
        this->writeInt32((int)this->layout.vehicles.size()); // Vehicle count
        for (const Vehicle &vehicle : layout.vehicles) {
            this->writeString(vehicle.display_name); // Display name
            this->writeVector2(vehicle.pos); // Position
            this->writeQuaternion(vehicle.rot); // Rotation
            this->writeString(vehicle.prefab_name); // Prefab name
            this->writeFloat(vehicle.target_speed); // Target speed
            this->writeFloat(vehicle.mass); // Mass
            this->writeFloat(vehicle.braking_force_multiplier); // Braking force multiplier
            this->writeInt32(vehicle.strength_method); // Strength method
            this->writeFloat(vehicle.acceleration); // Acceleration
            this->writeFloat(vehicle.max_slope); // Max slope
            this->writeFloat(vehicle.desired_acceleration); // Desired acceleration
            this->writeFloat(vehicle.shocks_multiplier); // Shocks multiplier
            this->writeFloat(vehicle.rotation_degrees); // Rotation degrees
            this->writeFloat(vehicle.time_delay); // Time delay (seconds)
            this->writeBool(vehicle.idle_on_downhill); // Idle on downhill
            this->writeBool(vehicle.flipped); // Flipped
            this->writeBool(vehicle.ordered_checkpoints); // Ordered checkpoints
            this->writeString(vehicle.guid); // GUID

            Vehicle v = this->findVehicleByGuid(vehicle.guid);
            this->writeInt32((int)v.checkpoint_guids.size()); // Checkpoint count
            for (const std::string &checkpoint_guid : v.checkpoint_guids) {
                this->writeString(checkpoint_guid); // Checkpoint GUID
            }
        }
        U::log_info_s("Serialized %s vehicles", U::intc((int)this->layout.vehicles.size()).c_str());

        // Vehicle stop triggers
        this->writeInt32((int)this->layout.vehicleStopTriggers.size()); // Vehicle stop trigger count
        for (const VehicleStopTrigger &trigger : layout.vehicleStopTriggers) {
            this->writeVector2(trigger.pos); // Position
            this->writeQuaternion(trigger.rot); // Rotation
            this->writeFloat(trigger.height); // Height
            this->writeFloat(trigger.rotation_degrees); // Rotation degrees
            this->writeBool(trigger.flipped); // Flipped
            this->writeString(trigger.prefab_name); // Prefab name
            this->writeString(trigger.stop_vehicle_guid); // Stop vehicle GUID
        }
        U::log_info_s("Serialized %s vehicle stop triggers", U::intc((int)this->layout.vehicleStopTriggers.size()).c_str());

        // Timelines
        this->writeInt32((int)this->layout.eventTimelines.size()); // Timeline count
        for (const EventTimeline &timeline : layout.eventTimelines) {
            this->writeString(timeline.checkpoint_guid); // Checkpoint GUID

            this->writeInt32((int)timeline.stages.size()); // Stage count
            for (const EventStage &stage : timeline.stages) {
                this->writeInt32((int)stage.units.size()); // Unit count
                for (const EventUnit &unit : stage.units) {
                    this->writeString(unit.guid); // GUID
                }
            }
        }
        U::log_info_s("Serialized %s timelines", U::intc((int)this->layout.eventTimelines.size()).c_str());

        // Checkpoints
        this->writeInt32((int)this->layout.checkpoints.size()); // Checkpoint count
        for (const Checkpoint &checkpoint : layout.checkpoints) {
            this->writeVector2(checkpoint.pos); // Position
            this->writeString(checkpoint.prefab_name); // Prefab name
            this->writeString(checkpoint.vehicle_guid); // Vehicle GUID
            this->writeString(checkpoint.vehicle_restart_phase_guid); // Vehicle restart phase GUID
            this->writeBool(checkpoint.trigger_timeline); // Trigger timeline
            this->writeBool(checkpoint.stop_vehicle); // Stop vehicle
            this->writeBool(checkpoint.reverse_vehicle_on_restart); // Reverse vehicle on restart
            this->writeString(checkpoint.guid); // GUID
        }
        U::log_info_s("Serialized %s checkpoints", U::intc((int)this->layout.checkpoints.size()).c_str());

        // Terrain stretches
        this->writeInt32((int)this->layout.terrainStretches.size()); // Terrain stretch count
        for (const TerrainIsland &stretch : layout.terrainStretches) {
            this->writeVector3(stretch.pos); // Position
            this->writeString(stretch.prefab_name); // Prefab name
            this->writeFloat(stretch.height_added); // Height added
            this->writeFloat(stretch.right_edge_water_height); // Right edge water height
            this->writeInt32(stretch.terrain_island_type); // Terrain island type
            this->writeInt32(stretch.variant_index); // Variant index
            this->writeBool(stretch.flipped); // Flipped
            this->writeBool(stretch.lock_position); // Lock position
        }
        U::log_info_s("Serialized %s terrain stretches", U::intc((int)this->layout.terrainStretches.size()).c_str());

        // Platforms
        this->writeInt32((int)this->layout.platforms.size()); // Platform count
        for (const Platform &platform : layout.platforms) {
            this->writeVector2(platform.pos); // Position
            this->writeFloat(platform.width); // Width
            this->writeFloat(platform.height); // Height
            this->writeBool(platform.flipped); // Flipped
            this->writeBool(platform.solid); // Solid
        }
        U::log_info_s("Serialized %s platforms", U::intc((int)this->layout.platforms.size()).c_str());

        // Ramps
        this->writeInt32((int)this->layout.ramps.size()); // Ramp count
        for (const Ramp &ramp : layout.ramps) {
            this->writeVector2(ramp.pos); // Position

            this->writeInt32((int)ramp.control_points.size()); // Control point count
            for (const Vec2 &control_point : ramp.control_points) {
                this->writeVector2(control_point); // Control point
            }

            this->writeFloat(ramp.height); // Height
            this->writeInt32(ramp.num_segments); // Num segments
            this->writeInt32(ramp.spline_type); // Spline type
            this->writeBool(ramp.flipped_vertical); // Flipped vertical
            this->writeBool(ramp.flipped_horizontal); // Flipped horizontal
            this->writeBool(ramp.hide_legs); // Hide legs
            this->writeBool(ramp.flipped_legs); // Flipped legs

            // Line points
            this->writeInt32((int)ramp.line_points.size());
            for (const Vec2 &line_point : ramp.line_points) {
                this->writeVector2(line_point);
            }
        }
        U::log_info_s("Serialized %s ramps", U::intc((int)this->layout.ramps.size()).c_str());

        // Vehicle restart phases
        this->writeInt32((int)this->layout.vehicleRestartPhases.size()); // Vehicle restart phase count
        for (const VehicleRestartPhase &phase : layout.vehicleRestartPhases) {
            this->writeFloat(phase.time_delay); // Time delay (seconds)
            this->writeString(phase.guid); // GUID
            this->writeString(phase.vehicle_guid); // Vehicle GUID
        }
        U::log_info_s("Serialized %s vehicle restart phases", U::intc((int)this->layout.vehicleRestartPhases.size()).c_str());

        // Flying objects
        this->writeInt32((int)this->layout.flyingObjects.size()); // Flying object count
        for (const FlyingObject &flying_object : layout.flyingObjects) {
            this->writeVector3(flying_object.pos); // Position
            this->writeVector3(flying_object.scale); // Scale
            this->writeString(flying_object.prefab_name); // Prefab name
        }
        U::log_info_s("Serialized %s flying objects", U::intc((int)this->layout.flyingObjects.size()).c_str());

        // Rocks
        this->writeInt32((int)this->layout.rocks.size()); // Rock count
        for (const Rock &rock : layout.rocks) {
            this->writeVector3(rock.pos); // Position
            this->writeVector3(rock.scale); // Scale
            this->writeString(rock.prefab_name); // Prefab name
            this->writeBool(rock.flipped); // Flipped
        }
        U::log_info_s("Serialized %s rocks", U::intc((int)this->layout.rocks.size()).c_str());

        // Water blocks
        this->writeInt32((int)this->layout.waterBlocks.size()); // Water block count
        for (const WaterBlock &water_block : layout.waterBlocks) {
            this->writeVector3(water_block.pos); // Position
            this->writeFloat(water_block.width); // Width
            this->writeFloat(water_block.height); // Height
            this->writeBool(water_block.lock_position); // Lock position
        }
        U::log_info_s("Serialized %s water blocks", U::intc((int)this->layout.waterBlocks.size()).c_str());

        // Budget
        this->writeInt32((int)this->layout.budget.cash); // Cash
        this->writeInt32((int)this->layout.budget.road); // Road
        this->writeInt32((int)this->layout.budget.wood); // Wood
        this->writeInt32((int)this->layout.budget.steel); // Steel
        this->writeInt32((int)this->layout.budget.hydraulics); // Hydraulics
        this->writeInt32((int)this->layout.budget.rope); // Rope
        this->writeInt32((int)this->layout.budget.cable); // Cable
        this->writeInt32((int)this->layout.budget.spring); // Spring
        this->writeInt32((int)this->layout.budget.bungee_rope); // Bungee rope
        this->writeBool(this->layout.budget.allow_wood); // Allow wood
        this->writeBool(this->layout.budget.allow_steel); // Allow steel
        this->writeBool(this->layout.budget.allow_hydraulics); // Allow hydraulics
        this->writeBool(this->layout.budget.allow_rope); // Allow rope
        this->writeBool(this->layout.budget.allow_cable); // Allow cable
        this->writeBool(this->layout.budget.allow_spring); // Allow spring
        this->writeBool(this->layout.budget.allow_reinforced_road); // Allow reinforced road
        U::log_info_s("Serialized budget of $%s", U::intc(this->layout.budget.cash, 0, 100000000, 0, 100000000).c_str());

        // Settings
        this->writeBool(this->layout.settings.hydraulics_controller_enabled); // Hydraulics controller enabled
        this->writeBool(this->layout.settings.unbreakable); // Unbreakable
        U::log_info_s("Serialized settings");

        // Custom shapes
        this->writeInt32((int)this->layout.customShapes.size()); // Custom shape count
        for (const CustomShape &cs : layout.customShapes) {
            this->writeVector3(cs.pos); // Position
            this->writeQuaternion(cs.rot); // Rotation
            this->writeVector3(cs.scale); // Scale
            this->writeBool(cs.flipped); // Flipped
            this->writeBool(cs.dynamic); // Dynamic
            this->writeBool(cs.collides_with_road); // Collides with road
            this->writeBool(cs.collides_with_nodes); // Collides with nodes
            this->writeBool(cs.collides_with_split_nodes); // Collides with split nodes
            this->writeFloat(cs.rotation_degrees); // Rotation degrees
            this->writeColor(cs.color); // Color
            this->writeFloat(cs.mass); // Mass
            this->writeFloat(cs.bounciness); // Bounciness
            this->writeFloat(cs.pin_motor_strength); // Pin motor strength
            this->writeFloat(cs.pin_target_velocity); // Pin target velocity

            this->writeInt32((int)cs.points_local_space.size()); // Point count
            for (const Vec2 &point : cs.points_local_space) {
                this->writeVector2(point); // Point
            }

            this->writeInt32((int)cs.static_pins.size()); // Static pin count
            for (const Vec3 &static_pin : cs.static_pins) {
                this->writeVector3(static_pin); // Static pin
            }

            this->writeInt32((int)cs.dynamic_anchor_guids.size()); // Dynamic anchor GUID count
            for (const std::string &dynamic_anchor_guid : cs.dynamic_anchor_guids) {
                this->writeString(dynamic_anchor_guid); // Dynamic anchor GUID
            }
        }
        U::log_info_s("Serialized %s custom shapes", U::intc((int)this->layout.customShapes.size()).c_str());

        // Workshop
        this->writeString(this->layout.workshop.id); // Workshop ID
        this->writeString(this->layout.workshop.leaderboard_id); // Workshop leaderboard ID
        this->writeString(this->layout.workshop.title); // Workshop title
        this->writeString(this->layout.workshop.description); // Workshop description
        this->writeBool(this->layout.workshop.autoplay); // Autoplay

        this->writeInt32((int)this->layout.workshop.tags.size()); // Workshop tag count
        for (const std::string &tag : this->layout.workshop.tags) {
            this->writeString(tag); // Tag
        }
        U::log_info_s(
            layout.workshop.title.empty()
            ? "Serialized workshop"
            : "Serialized workshop level '%s'", ("\x1B[1;95m" + layout.workshop.title + "\x1B[0m").c_str()
        );

        // Support pillars
        this->writeInt32((int)this->layout.supportPillars.size()); // Support pillar count
        for (const SupportPillar &support_pillar : layout.supportPillars) {
            this->writeVector3(support_pillar.pos); // Position
            this->writeVector3(support_pillar.scale); // Scale
            this->writeString(support_pillar.prefab_name); // Prefab name
        }
        U::log_info_s("Serialized %s support pillars", U::intc((int)this->layout.supportPillars.size()).c_str());

        // Pillars
        this->writeInt32((int)this->layout.pillars.size()); // Pillar count
        for (const Pillar &pillar : layout.pillars) {
            this->writeVector3(pillar.pos); // Position
            this->writeFloat(pillar.height); // Height
            this->writeString(pillar.prefab_name); // Prefab name
        }
        U::log_info_s("Serialized %s pillars", U::intc((int)this->layout.pillars.size()).c_str());

        this->file << std::flush;
    }
};

class SimpleBridgeDeserializer {
public:
    char* bytes;
    int offset{};
    explicit SimpleBridgeDeserializer(char* bytes) {
        this->bytes = bytes;
    }
    Bridge deserializeBridge() {
        Bridge bridge;

        bridge.version = this->readInt32();
        U::log_info_d("Bridge version: %s", U::intc(bridge.version).c_str());
        if (bridge.version > MAX_BRIDGE_VERSION) {
            Utils::log_warn_d("Bridge saved with a newer version of the bridge format. This may cause problems.");
        }

        if (bridge.version < 2) {
            return bridge;  // v2- not supported (neither in game)
        }

        // Joints
        int num = this->readInt32();
        U::log_info_d("Deserializing %s joints", U::intc(num).c_str());
        for (int i = 0; i < num; i++) {
            BridgeJoint joint;
            joint.pos = this->readVector3();
            joint.is_anchor = this->readBool();
            joint.is_split = this->readBool();
            joint.guid = this->readString();
            bridge.joints.push_back(joint);
        }

        // Edges
        num = this->readInt32();
        U::log_info_d("Deserializing %s edges", U::intc(num).c_str());
        for (int i = 0; i < num; i++) {
            BridgeEdge edge;
            edge.material_type = (BridgeMaterialType)this->readInt32();
            edge.node_a_guid = this->readString();
            edge.node_b_guid = this->readString();
            edge.joint_a_part = (SplitJointPart)this->readInt32();
            edge.joint_b_part = (SplitJointPart)this->readInt32();
        }

        // Springs in v7+
        if (bridge.version >= 7) {
            num = this->readInt32();
            U::log_info_d("Deserializing %s springs", U::intc(num).c_str());
            for (int i = 0; i < num; i++) {
                BridgeSpring spring;
                spring.normalized_value = this->readFloat();
                spring.node_a_guid = this->readString();
                spring.node_b_guid = this->readString();
                spring.guid = this->readString();
                bridge.springs.push_back(spring);
            }
        }

        // Pistons
        num = this->readInt32();
        U::log_info_d("Deserializing %s pistons", U::intc(num).c_str());
        for (int i = 0; i < num; i++) {
            Piston piston;
            piston.normalized_value = this->readFloat();
            piston.node_a_guid = this->readString();
            piston.node_b_guid = this->readString();
            piston.guid = this->readString();

            if (bridge.version < 8) {
                piston.normalized_value = Deserializer::fixPistonNormalizedValue(piston.normalized_value);
            }

            bridge.pistons.push_back(piston);
        }

        // Hydraulic phases
        num = this->readInt32();
        U::log_info_d("Deserializing %s hydraulic phases", U::intc(num).c_str());
        for (int i = 0; i < num; i++) {
            HydraulicsControllerPhase phase;
            phase.hydraulics_phase_guid = this->readString();

            // piston guids
            int num_pistons = this->readInt32();
            for (int j = 0; j < num_pistons; j++) {
                phase.piston_guids.push_back(this->readString());
            }

            if (bridge.version > 2) {
                int split_joint_count = this->readInt32();
                for (int j = 0; j < split_joint_count; j++) {
                    BridgeSplitJoint split_joint;
                    split_joint.guid = this->readString();
                    split_joint.state = (SplitJointState)this->readInt32();
                    phase.bridge_split_joints.push_back(split_joint);
                }
            } else {
                // garbage data
                int count = this->readInt32();
                for (int j = 0; j < count; j++) {
                    this->readString();
                }
            }

            if (bridge.version > 9) {
                phase.disable_new_additions = this->readBool();
            }
        }

        // Garbage data (v5)
        if (bridge.version == 5) {
            int count = this->readInt32();
            for (int i = 0; i < count; i++) {
                this->readString();
            }
        }

        // Anchors (v6+)
        if (bridge.version >= 6) {
            num = this->readInt32();
            U::log_info_d("Deserializing %s anchors", U::intc(num).c_str());
            for (int i = 0; i < num; i++) {
                BridgeJoint anchor;
                anchor.pos = this->readVector3();
                anchor.is_anchor = this->readBool();
                anchor.is_split = this->readBool();
                anchor.guid = this->readString();
                bridge.anchors.push_back(anchor);
            }
        }

        // Random bool at end (v4-v8)
        if (bridge.version >= 4 && bridge.version < 9) {
            this->readBool();
        }

        return bridge;
    }
private:
    char* readByte() {
        char* b = &this->bytes[this->offset];
        this->offset++;
        return b;
    }
    char* readBytes(int count) {
        char* b = new char[count];
        for (int i = 0; i < count; i++) {
            b[i] = this->bytes[this->offset + i];
        }
        this->offset += count;
        return b;
    }
    bool readBool() {
        bool value = *reinterpret_cast<bool *>(this->readByte());
        return value;
    }
    int16_t readInt16() {
        int16_t value = *reinterpret_cast<int16_t *>(this->readBytes(sizeof(int16_t)));
        return value;
    }
    uint16_t readUInt16() {
        uint16_t value = *reinterpret_cast<uint16_t *>(this->readBytes(sizeof(uint16_t)));
        return value;
    }
    int32_t readInt32() {
        int32_t value = *reinterpret_cast<int32_t *>(this->readBytes(sizeof(int32_t)));
        return value;
    }
    float readFloat() {
        float value = *reinterpret_cast<float *>(this->readBytes(sizeof(float)));
        return value;
    }
    std::string readString() {
        int length = this->readUInt16();
        char* data = this->readBytes(length);
        std::string str(data, length);
        delete[] data;
        return str;
    }
    Vec2 readVector2() {
        Vec2 value{};
        value.x = this->readFloat();
        value.y = this->readFloat();
        return value;
    }
    Vec3 readVector3() {
        Vec3 value{};
        value.x = this->readFloat();
        value.y = this->readFloat();
        value.z = this->readFloat();
        return value;
    }
};

class SlotDeserializer {
public:
    std::string path;
    std::ifstream file;
    explicit SlotDeserializer (const std::string &path) {
        this->path = path;
        this->file.open(path, std::ios::binary);
        if (!this->file.is_open()) {
            U::log_error_s("Failed to open file '%s'", path.c_str());
            exit(1);
        }
    }
    SaveSlot deserializeSlot() {
        // So, a bit on how this works:
        //   This is basically an *extremely* condensed version of OdinSerializer.
        //   Since we don't have the correct types available to us, we have a custom
        //   override, which is the BridgeSaveSlotData struct. As this code is only
        //   meant for a specific purpose, we don't have to deserialize in the
        //   conventional way. Instead, we just read the data we expect, and ensure
        //   it's correct, which may mean same name, same type, etc.
        //   We aren't worrying about nodes here, since we don't need to act like
        //   this is multilevel data.
        //   One thing about this format is that it's very type-specific. For example,
        //   it's not possible to deserialize a string as a float. This is because the
        //   type, assuming a built-in, is read from a single byte, and ensured it's
        //   in the BinaryEntryType enum.

        // create a SaveSlot object
        SaveSlot slot;

        enterNode();  // enter the root node

        // First, read the version number
        EntryTypeReturn et = this->peekEntryType();
        assert(et.type == EntryType::Integer);  // make sure it's an integer
        assert(et.name == "m_Version");
        int version = this->readInt(); // read the version number
        U::log_info_d("Save slot version: " + U::intc(version));
        // warn the user if the version is greater than the current max fully supported version
        if (version > MAX_SLOT_VERSION) {
            Utils::log_warn_d("Slot saved with a newer version of the slot format. This may cause problems.");
        }
        slot.version = version;

        // Next, read the physics version
        et = this->peekEntryType();
        assert(et.type == EntryType::Integer);  // make sure it's an integer
        assert(et.name == "m_PhysicsVersion");
        int physics_version = this->readInt(); // read the physics version number
        U::log_info_d("Save slot physics version: " + U::intc(physics_version));
        // warn the user if the physics version is greater than the current max fully supported physics version
        if (physics_version > MAX_PHYSICS_VERSION) {
            U::log_warn_d("Save slot physics version is greater than the current max fully supported physics version, bugs may occur");
        }
        slot.physicsVersion = physics_version;

        // Next, the slot ID
        et = this->peekEntryType();
        assert(et.type == EntryType::Integer);  // make sure it's an integer
        assert(et.name == "m_SlotID");
        int slot_id = this->readInt(); // read the slot ID
        U::log_info_d("Save slot ID: " + U::intc(slot_id));
        slot.slotId = slot_id;

        // Next, the slot display name
        et = this->peekEntryType();
        assert(et.type == EntryType::String);  // make sure it's a string
        assert(et.name == "m_DisplayName");
        std::string slot_name = this->readString(); // read the slot name
        U::log_info_d("Save slot name: " + slot_name);
        slot.displayName = slot_name;

        // Next, the slot filename-o ./brokenslot slots/LongDrawbridge_Auto-Save.slot
        et = this->peekEntryType();
        assert(et.type == EntryType::String);  // make sure it's a string
        assert(et.name == "m_SlotFilename");
        std::string slot_filename = this->readString(); // read the slot filename
        U::log_info_d("Save slot filename: " + slot_filename);
        slot.fileName = slot_filename;

        // Next, the budget (price of bridge)
        et = this->peekEntryType();
        assert(et.type == EntryType::Integer);  // make sure it's an integer
        assert(et.name == "m_Budget");
        int budget = this->readInt(); // read the budget
        U::log_info_d("Save slot budget: $" + U::intc(budget, 0, 10000000, 0, 10000000));
        slot.budget = budget;

        // Next, the last write time in C# ticks
        et = this->peekEntryType();
        assert(et.type == EntryType::Integer);  // make sure it's a long (considered an integer in EntryType)
        assert(et.name == "m_LastWriteTimeTicks");
        long last_write_time = this->readLong(); // read the last write time
        U::log_info_d("Save slot last write time: " + U::ticks_to_datetime(last_write_time));
        slot.lastWriteTimeTicks = last_write_time;

        // Next, the bridge, which is a bit weird
        // Enter the node
        enterNode();

        // read the bridge data
        et = this->peekEntryType();
        assert(et.type == EntryType::PrimitiveArrayType);  // make sure it's a primitive array
        int num = this->readInt();
        int num2 = this->readInt();
        int num3 = num * num2;
        // read num3 bytes from the stream
        char *bridge_data = new char[num3];
        this->file.read(bridge_data, num3);

        U::log_info_d("Loading bridge data of size %s...", U::intc(num3, 0, 100000000, 0, 100000000).c_str());
        SimpleBridgeDeserializer bd(bridge_data);
        Bridge bridge = bd.deserializeBridge();
        U::log_info_d("Bridge loaded");
        slot.bridge = bridge;

        // make sure we are at an end of node
        et = this->peekEntryType();
        assert(et.type == EntryType::EndOfNodeType);
        U::log_info_d("Exiting node");

        // thumbnail data
        et = this->peekEntryType();
        assert(et.name == "m_Thumb");
        if (et.type == EntryType::Null) {
            U::log_info_d("No thumbnail in save slot");
        } else {
            // for some reason, there is a type ID defining byte here
            this->file.get();
            // typename
            this->readInt();
            // node ID
            int node_id = this->readInt();
            U::log_info_d("Entering node id " + std::to_string(node_id));

            et = this->peekEntryType();
            assert(et.type == EntryType::PrimitiveArrayType);  // make sure it's a primitive array
            num = this->readInt();
            num2 = this->readInt();
            num3 = num * num2;
            U::log_info_d("Thumbnail data size: " + U::intc(num3, 0, 10000000, 0, 10000000));
            char* thumbnail_data = new char[num3];
            this->file.read(thumbnail_data, num3);
            slot.thumbnail = thumbnail_data;

            et = this->peekEntryType();
            assert(et.type == EntryType::EndOfNodeType);
            U::log_info_d("Exiting node");
        }

        // If the layout uses unlimited materials
        et = this->peekEntryType();
        assert(et.name == "m_UsingUnlimitedMaterials");
        assert(et.type == EntryType::Boolean);
        bool unlimited_materials = this->readBool();
        U::log_info_d("Unlimited materials: %s", unlimited_materials ? "\x1B[1;92myes\x1B[0m" : "\x1B[1;91mno\x1B[0m");
        slot.unlimitedMaterials = unlimited_materials;

        // If the layout uses unlimited budget
        et = this->peekEntryType();
        assert(et.name == "m_UsingUnlimitedBudget");
        assert(et.type == EntryType::Boolean);
        bool unlimited_budget = this->readBool();
        U::log_info_d("Unlimited budget: %s", unlimited_budget ? "\x1B[1;92myes\x1B[0m" : "\x1B[1;91mno\x1B[0m");
        slot.unlimitedBudget = unlimited_budget;

        // End of node
        et = this->peekEntryType();
        assert(et.type == EntryType::EndOfNodeType);
        U::log_info_d("Exiting node");

        return slot;
    }
    ~SlotDeserializer() {
        this->file.close();
    }
private:
    int readInt() {
        int i;
        this->file.read(reinterpret_cast<char *>(&i), sizeof(int));
        return i;
    }
    std::string readString() {
        int num = this->file.get();
        if (num < 0) {
            return "";
        }
        if (num == 0) {
            int num2 = this->readInt();
            char* bytes = new char[num2];
            this->file.read(bytes, num2);
            std::string str(bytes, num2);
            delete[] bytes;
            return str;
        }
        if (num == 1) {
            int length = this->readInt();
            std::u16string str(length, '\0');
            this->file.read((char*)&str[0], length * 2);  // null spaced
            // convert u16 to u8
            std::wstring_convert<std::codecvt_utf8_utf16<char16_t>,char16_t> convert;
            return std::string(convert.to_bytes(str));
        }
        return "";
    }
    EntryTypeReturn peekEntryType() {
        int c = this->file.get();
        if (c == EOF) {
            return EntryTypeReturn{EntryType::EndOfStreamType, ""};
        }
        auto bt = (BinaryEntryType)c;
        std::string name;

        switch (bt) {
            case BinaryEntryType::NamedStartOfReferenceNode:
            case BinaryEntryType::NamedStartOfStructNode:
                name = this->readString();
                return EntryTypeReturn{EntryType::StartOfNode, name};
                break;
            case BinaryEntryType::UnnamedStartOfReferenceNode:
            case BinaryEntryType::UnnamedStartOfStructNode:
                return EntryTypeReturn{EntryType::StartOfNode, ""};
                break;
            case BinaryEntryType::EndOfNode:
                return EntryTypeReturn{EntryType::EndOfNodeType, ""};
                break;
            case BinaryEntryType::StartOfArray:
                return EntryTypeReturn{EntryType::StartOfArrayType, ""};
                break;
            case BinaryEntryType::EndOfArray:
                return EntryTypeReturn{EntryType::EndOfArrayType, ""};
                break;
            case BinaryEntryType::PrimitiveArray:
                return EntryTypeReturn{EntryType::PrimitiveArrayType, ""};
                break;
            case BinaryEntryType::NamedInternalReference:
                name = this->readString();
                return EntryTypeReturn{EntryType::InternalReference, name};
                break;
            case BinaryEntryType::UnnamedInternalReference:
                return EntryTypeReturn{EntryType::InternalReference, ""};
                break;
            case BinaryEntryType::NamedExternalReferenceByIndex:
                name = this->readString();
                return EntryTypeReturn{EntryType::ExternalReferenceByIndex, name};
                break;
            case BinaryEntryType::UnnamedExternalReferenceByIndex:
                return EntryTypeReturn{EntryType::ExternalReferenceByIndex, ""};
                break;
            case BinaryEntryType::NamedExternalReferenceByGuid:
                name = this->readString();
                return EntryTypeReturn{EntryType::ExternalReferenceByGuid, name};
                break;
            case BinaryEntryType::UnnamedExternalReferenceByGuid:
                return EntryTypeReturn{EntryType::ExternalReferenceByGuid, ""};
                break;
            case BinaryEntryType::NamedSByte:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedSByte:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedByte:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedByte:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedShort:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedShort:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedUShort:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedUShort:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedInt:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedInt:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedUInt:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedUInt:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedLong:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedLong:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedULong:
                name = this->readString();
                return EntryTypeReturn{EntryType::Integer, name};
                break;
            case BinaryEntryType::UnnamedULong:
                return EntryTypeReturn{EntryType::Integer, ""};
                break;
            case BinaryEntryType::NamedFloat:
                name = this->readString();
                return EntryTypeReturn{EntryType::FloatingPoint, name};
                break;
            case BinaryEntryType::UnnamedFloat:
                return EntryTypeReturn{EntryType::FloatingPoint, ""};
                break;
            case BinaryEntryType::NamedDouble:
                name = this->readString();
                return EntryTypeReturn{EntryType::FloatingPoint, name};
                break;
            case BinaryEntryType::UnnamedDouble:
                return EntryTypeReturn{EntryType::FloatingPoint, ""};
                break;
            case BinaryEntryType::NamedDecimal:
                name = this->readString();
                return EntryTypeReturn{EntryType::FloatingPoint, name};
                break;
            case BinaryEntryType::UnnamedDecimal:
                return EntryTypeReturn{EntryType::FloatingPoint, ""};
                break;
            case BinaryEntryType::NamedChar:
                name = this->readString();
                return EntryTypeReturn{EntryType::String, name};
                break;
            case BinaryEntryType::UnnamedChar:
                return EntryTypeReturn{EntryType::String, ""};
                break;
            case BinaryEntryType::NamedString:
                name = this->readString();
                return EntryTypeReturn{EntryType::String, name};
                break;
            case BinaryEntryType::UnnamedString:
                return EntryTypeReturn{EntryType::String, ""};
                break;
            case BinaryEntryType::NamedGuid:
                name = this->readString();
                return EntryTypeReturn{EntryType::Guid, name};
                break;
            case BinaryEntryType::UnnamedGuid:
                return EntryTypeReturn{EntryType::Guid, ""};
                break;
            case BinaryEntryType::NamedBoolean:
                name = this->readString();
                return EntryTypeReturn{EntryType::Boolean, name};
                break;
            case BinaryEntryType::UnnamedBoolean:
                return EntryTypeReturn{EntryType::Boolean, ""};
                break;
            case BinaryEntryType::NamedNull:
                name = this->readString();
                return EntryTypeReturn{EntryType::Null, name};
                break;
            case BinaryEntryType::UnnamedNull:
                return EntryTypeReturn{EntryType::Null, ""};
                break;
            case BinaryEntryType::TypeName:
            case BinaryEntryType::TypeID:
                U::log_error_d("BinaryEntryType::TypeName or BinaryEntryType::TypeID cannot be peeked");
                exit(1);
                break;
            case BinaryEntryType::EndOfStream:
                return EntryTypeReturn{EntryType::EndOfStreamType, ""};
                break;
            case BinaryEntryType::NamedExternalReferenceByString:
                name = this->readString();
                return EntryTypeReturn{EntryType::ExternalReferenceByString, name};
                break;
            case BinaryEntryType::UnnamedExternalReferenceByString:
                return EntryTypeReturn{EntryType::ExternalReferenceByString, ""};
                break;
            default:
                U::log_error_d("Unknown BinaryEntryType: " + std::to_string(bt));
                return EntryTypeReturn{EntryType::InvalidType, ""};
                break;
        }
    }
    TypeEntryReturn readTypeEntry() {
        int num = this->file.get();
        if (num < 0) {
            return {};
        }
        auto bt = (BinaryEntryType)num;
        if (bt == BinaryEntryType::TypeName) {
            int key = this->readInt();
            std::string type_name = this->readString();
            // split by ', ' to get the type name and the assembly name
            std::vector<std::string> type_names;
            std::stringstream ss(type_name);
            std::string item;
            while (std::getline(ss, item, ',')) {
                type_names.push_back(item);
            }

            if (type_names.size() != 2) {
                U::log_error_d("Type name is not in the format of type_name, assembly_name");
                exit(1);
            }
            if (type_names[0] == "BridgeSaveSlotData") {
                U::log_info_d("Using override for type BridgeSaveSlotData");
            }

            // remove the space before the assembly name
            type_names[1].erase(0, 1);

            return TypeEntryReturn{type_names[0], type_names[1]};
        } else if (bt == BinaryEntryType::TypeID) {
            U::log_info_d("Type ID read, will assume override in deserializer is present and ignore this value");
        } else {
            U::log_error_d("Unknown type entry flag: " + std::to_string(bt));
            exit(1);
        }
        return {};
    }
    bool readBool() {
        bool res = this->file.get() == 1;
        return res;
    }
    long readLong() {
        uint64_t  value =
            static_cast<uint64_t>(this->file.get()) |
            static_cast<uint64_t>(this->file.get()) << 8 |
            static_cast<uint64_t>(this->file.get()) << 16 |
            static_cast<uint64_t>(this->file.get()) << 24 |
            static_cast<uint64_t>(this->file.get()) << 32 |
            static_cast<uint64_t>(this->file.get()) << 40 |
            static_cast<uint64_t>(this->file.get()) << 48 |
            static_cast<uint64_t>(this->file.get()) << 56;
        return static_cast<long>(value);
    }
    void enterNode() {
        EntryTypeReturn et = this->peekEntryType();
        if (et.type == EntryType::StartOfNode) {
            TypeEntryReturn type = this->readTypeEntry();
            int id = this->readInt();
            Utils::log_info_d("Entering node id " + std::to_string(id));
        }
    }
};

void dump_json(Layout &layout, const std::string& path) {
    // TODO: make this neater and avoid these repetitive for loops
    json j;
    j["m_Version"] = layout.version;
    j["m_ThemeStubKey"] = layout.stubKey;
    j["m_Anchors"] = json::array();
    for (const auto &anchor : layout.anchors) {
        json anchor_json;
        anchor_json["m_Pos"]["x"] = anchor.pos.x;
        anchor_json["m_Pos"]["y"] = anchor.pos.y;
        anchor_json["m_Pos"]["z"] = anchor.pos.z;
        anchor_json["m_IsAnchor"] = anchor.is_anchor;
        anchor_json["m_IsSplit"] = anchor.is_split;
        anchor_json["m_Guid"] = anchor.guid;
        j["m_Anchors"].push_back(anchor_json);
    }
    j["m_HydraulicPhases"] = json::array();
    for (const auto &phase : layout.phases) {
        json phase_json;
        phase_json["m_TimeDelaySeconds"] = phase.time_delay;
        phase_json["m_Guid"] = phase.guid;
        j["m_UndoGuid"] = nullptr;  // For compatibility with PolyConverter
        j["m_HydraulicPhases"].push_back(phase_json);
    }
    auto bridge = j["m_Bridge"];
    bridge["m_Version"] = layout.bridge.version;
    bridge["m_BridgeJoints"] = json::array();
    for (const auto &joint : layout.bridge.joints) {
        json joint_json;
        joint_json["m_Pos"]["x"] = joint.pos.x;
        joint_json["m_Pos"]["y"] = joint.pos.y;
        joint_json["m_Pos"]["z"] = joint.pos.z;
        joint_json["m_IsAnchor"] = joint.is_anchor;
        joint_json["m_IsSplit"] = joint.is_split;
        joint_json["m_Guid"] = joint.guid;
        bridge["m_BridgeJoints"].push_back(joint_json);
    }
    bridge["m_BridgeEdges"] = json::array();
    for (const auto &edge : layout.bridge.edges) {
        json edge_json;
        edge_json["m_Material"] = edge.material_type;
        edge_json["m_NodeA_Guid"] = edge.node_a_guid;
        edge_json["m_NodeB_Guid"] = edge.node_b_guid;
        edge_json["m_JointAPart"] = edge.joint_a_part;
        edge_json["m_JointBPart"] = edge.joint_b_part;
        bridge["m_BridgeEdges"].push_back(edge_json);
    }
    bridge["m_BridgeSprings"] = json::array();
    for (const auto &spring : layout.bridge.springs) {
        json spring_json;
        spring_json["m_Guid"] = spring.guid;
        spring_json["m_NodeA_Guid"] = spring.node_a_guid;
        spring_json["m_NodeB_Guid"] = spring.node_b_guid;
        spring_json["m_NormalizedValue"] = spring.normalized_value;
        bridge["m_BridgeSprings"].push_back(spring_json);
    }
    bridge["m_Pistons"] = json::array();
    for (const auto &piston : layout.bridge.pistons) {
        json piston_json;
        piston_json["m_Guid"] = piston.guid;
        piston_json["m_NodeA_Guid"] = piston.node_a_guid;
        piston_json["m_NodeB_Guid"] = piston.node_b_guid;
        piston_json["m_NormalizedValue"] = piston.normalized_value;
        bridge["m_Pistons"].push_back(piston_json);
    }
    auto phases = bridge["m_HydraulicsController"]["m_Phases"] = json::array();
    for (const auto &phase : layout.bridge.phases) {
        json phase_json;
        phase_json["m_HydraulicsPhaseGuid"] = phase.hydraulics_phase_guid;
        phase_json["m_PistonGuids"] = phase.piston_guids;
        phase_json["m_BridgeSplitJoints"] = json::array();
        for (const auto &joint : phase.bridge_split_joints) {
            json joint_json;
            joint_json["m_BridgeJointGuid"] = joint.guid;
            joint_json["m_SplitJointState"] = joint.state;
            phase_json["m_BridgeSplitJoints"].push_back(joint_json);
        }
        phase_json["m_DisableNewAdditions"] = phase.disable_new_additions;
        phases.push_back(phase_json);
    }
    bridge["m_HydraulicsController"]["m_Phases"] = phases;
    bridge["m_Anchors"] = json::array();
    for (const auto &anchor : layout.bridge.anchors) {
        json anchor_json;
        anchor_json["m_Guid"] = anchor.guid;
        anchor_json["m_Pos"]["x"] = anchor.pos.x;
        anchor_json["m_Pos"]["y"] = anchor.pos.y;
        anchor_json["m_Pos"]["z"] = anchor.pos.z;
        anchor_json["m_IsAnchor"] = anchor.is_anchor;
        anchor_json["m_IsSplit"] = anchor.is_split;
        bridge["m_Anchors"].push_back(anchor_json);
    }

    j["m_Bridge"] = bridge;

    j["m_ZedAxisVehicles"] = json::array();
    for (const auto &zAxisVehicle : layout.zAxisVehicles) {
        json zAxisVehicle_json;
        zAxisVehicle_json["m_Guid"] = zAxisVehicle.guid;
        zAxisVehicle_json["m_Pos"]["x"] = zAxisVehicle.pos.x;
        zAxisVehicle_json["m_Pos"]["y"] = zAxisVehicle.pos.y;
        zAxisVehicle_json["m_TimeDelaySeconds"] = zAxisVehicle.time_delay;
        zAxisVehicle_json["m_PrefabName"] = zAxisVehicle.prefab_name;
        zAxisVehicle_json["m_Speed"] = zAxisVehicle.speed;
        zAxisVehicle_json["m_Rot"]["x"] = zAxisVehicle.rot.x;
        zAxisVehicle_json["m_Rot"]["y"] = zAxisVehicle.rot.y;
        zAxisVehicle_json["m_Rot"]["z"] = zAxisVehicle.rot.z;
        zAxisVehicle_json["m_Rot"]["w"] = zAxisVehicle.rot.w;
        zAxisVehicle_json["m_RotationDegrees"] = zAxisVehicle.rotation_degrees;
        j["m_ZedAxisVehicles"].push_back(zAxisVehicle_json);
    }

    j["m_Vehicles"] = json::array();
    for (const auto &vehicle : layout.vehicles) {
        json vehicle_json;
        vehicle_json["m_Guid"] = vehicle.guid;
        vehicle_json["m_Pos"]["x"] = vehicle.pos.x;
        vehicle_json["m_Pos"]["y"] = vehicle.pos.y;
        vehicle_json["m_Rot"]["x"] = vehicle.rot.x;
        vehicle_json["m_Rot"]["y"] = vehicle.rot.y;
        vehicle_json["m_Rot"]["z"] = vehicle.rot.z;
        vehicle_json["m_Rot"]["w"] = vehicle.rot.w;
        vehicle_json["m_PrefabName"] = vehicle.prefab_name;
        vehicle_json["m_TimeDelaySeconds"] = vehicle.time_delay;
        vehicle_json["m_PrefabName"] = vehicle.prefab_name;
        vehicle_json["m_CheckpointGuids"] = vehicle.checkpoint_guids;
        vehicle_json["m_Acceleration"] = vehicle.acceleration;
        vehicle_json["m_Mass"] = vehicle.mass;
        vehicle_json["m_BrakingForceMultiplier"] = vehicle.braking_force_multiplier;
        vehicle_json["m_StrengthMethod"] = vehicle.strength_method;
        vehicle_json["m_MaxSlope"] = vehicle.max_slope;
        vehicle_json["m_DesiredAcceleration"] = vehicle.desired_acceleration;
        vehicle_json["m_IdleOnDownhill"] = vehicle.idle_on_downhill;
        vehicle_json["m_Flipped"] = vehicle.flipped;
        vehicle_json["m_OrderedCheckpoints"] = vehicle.ordered_checkpoints;
        vehicle_json["m_DisplayName"] = vehicle.display_name;
        vehicle_json["m_RotationDegrees"] = vehicle.rotation_degrees;
        vehicle_json["m_TargetSpeed"] = vehicle.target_speed;
        vehicle_json["m_UndoGuid"] = nullptr;
        j["m_Vehicles"].push_back(vehicle_json);
    }

    j["m_VehicleStopTriggers"] = json::array();
    for (const auto &stopTrigger : layout.vehicleStopTriggers) {
        json stop_json;
        stop_json["m_Pos"]["x"] = stopTrigger.pos.x;
        stop_json["m_Pos"]["y"] = stopTrigger.pos.y;
        stop_json["m_Rot"]["x"] = stopTrigger.rot.x;
        stop_json["m_Rot"]["y"] = stopTrigger.rot.y;
        stop_json["m_Rot"]["z"] = stopTrigger.rot.z;
        stop_json["m_Rot"]["w"] = stopTrigger.rot.w;
        stop_json["m_PrefabName"] = stopTrigger.prefab_name;
        stop_json["m_Height"] = stopTrigger.height;
        stop_json["m_RotationDegrees"] = stopTrigger.rotation_degrees;
        stop_json["m_StopVehicleGuid"] = stopTrigger.stop_vehicle_guid;
        stop_json["m_Flipped"] = stopTrigger.flipped;
        stop_json["m_UndoGuid"] = nullptr;
        j["m_VehicleStopTriggers"].push_back(stop_json);
    }

    j["m_EventTimelines"] = json::array();
    for (const auto &timeline : layout.eventTimelines) {
        json timeline_json;
        timeline_json["m_CheckpointGuid"] = timeline.checkpoint_guid;
        timeline_json["m_Stages"] = json::array();
        for (const auto &stage : timeline.stages) {
            json stage_json;
            for (const auto &unit : stage.units) {
                json unit_json;
                unit_json["m_Guid"] = unit.guid;
                stage_json["m_Units"].push_back(unit_json);
            }
            timeline_json["m_Stages"].push_back(stage_json);
        }
        j["m_EventTimelines"].push_back(timeline_json);
    }

    j["m_Checkpoints"] = json::array();
    for (const auto &checkpoint : layout.checkpoints) {
        json checkpoint_json;
        checkpoint_json["m_Guid"] = checkpoint.guid;
        checkpoint_json["m_Pos"]["x"] = checkpoint.pos.x;
        checkpoint_json["m_Pos"]["y"] = checkpoint.pos.y;
        checkpoint_json["m_PrefabName"] = checkpoint.prefab_name;
        checkpoint_json["m_VehicleGuid"] = checkpoint.vehicle_guid;
        checkpoint_json["m_VehicleRestartPhaseGuid"] = checkpoint.vehicle_restart_phase_guid;
        checkpoint_json["m_TriggerTimeline"] = checkpoint.trigger_timeline;
        checkpoint_json["m_StopVehicle"] = checkpoint.stop_vehicle;
        checkpoint_json["m_ReverseVehicleOnRestart"] = checkpoint.reverse_vehicle_on_restart;
        checkpoint_json["m_UndoGuid"] = nullptr;
        j["m_Checkpoints"].push_back(checkpoint_json);
    }

    j["m_TerrainStretches"] = json::array();
    for (const auto &stretch : layout.terrainStretches) {
        json stretch_json;
        stretch_json["m_Pos"]["x"] = stretch.pos.x;
        stretch_json["m_Pos"]["y"] = stretch.pos.y;
        stretch_json["m_Pos"]["z"] = stretch.pos.z;
        stretch_json["m_PrefabName"] = stretch.prefab_name;
        stretch_json["m_HeightAdded"] = stretch.height_added;
        stretch_json["m_RightEdgeWaterHeight"] = stretch.right_edge_water_height;
        stretch_json["m_TerrainIslandType"] = stretch.terrain_island_type;
        stretch_json["m_VariantIndex"] = stretch.variant_index;
        stretch_json["m_Flipped"] = stretch.flipped;
        stretch_json["m_LockPosition"] = stretch.lock_position;
        stretch_json["m_UndoGuid"] = nullptr;
        j["m_TerrainStretches"].push_back(stretch_json);
    }

    j["m_Pillars"] = json::array();
    for (const auto &pillar : layout.pillars) {
        json pillar_json;
        pillar_json["m_Pos"]["x"] = pillar.pos.x;
        pillar_json["m_Pos"]["y"] = pillar.pos.y;
        pillar_json["m_Pos"]["z"] = pillar.pos.z;
        pillar_json["m_PrefabName"] = pillar.prefab_name;
        pillar_json["m_Height"] = pillar.height;
        pillar_json["m_UndoGuid"] = nullptr;
        j["m_Pillars"].push_back(pillar_json);
    }

    j["m_Platforms"] = json::array();
    for (const auto &platform : layout.platforms) {
        json platform_json;
        platform_json["m_Pos"]["x"] = platform.pos.x;
        platform_json["m_Pos"]["y"] = platform.pos.y;
        platform_json["m_Height"] = platform.height;
        platform_json["m_Width"] = platform.width;
        platform_json["m_Flipped"] = platform.flipped;
        platform_json["m_Solid"] = platform.solid;
        platform_json["m_UndoGuid"] = nullptr;
        j["m_Platforms"].push_back(platform_json);
    }

    j["m_Ramps"] = json::array();
    for (const auto &ramp : layout.ramps) {
        json ramp_json;
        ramp_json["m_Pos"]["x"] = ramp.pos.x;
        ramp_json["m_Pos"]["y"] = ramp.pos.y;
        ramp_json["m_Height"] = ramp.height;
        ramp_json["m_FlippedVertical"] = ramp.flipped_vertical;
        ramp_json["m_FlippedHorizontal"] = ramp.flipped_horizontal;
        ramp_json["m_FlippedLegs"] = ramp.flipped_legs;
        ramp_json["m_HideLegs"] = ramp.hide_legs;
        ramp_json["m_SplineType"] = ramp.spline_type;
        ramp_json["m_NumSegments"] = ramp.num_segments;
        ramp_json["m_UndoGuid"] = nullptr;
        for (Vec2 pos : ramp.control_points) {
            json point_json;
            point_json["x"] = pos.x;
            point_json["y"] = pos.y;
            ramp_json["m_ControlPoints"].push_back(point_json);
        }
        for (Vec2 pos : ramp.line_points) {
            json point_json;
            point_json["x"] = pos.x;
            point_json["y"] = pos.y;
            ramp_json["m_LinePoints"].push_back(point_json);
        }
        j["m_Ramps"].push_back(ramp_json);
    }

    j["m_VehicleRestartPhases"] = json::array();
    for (const auto &phase : layout.vehicleRestartPhases) {
        json phase_json;
        phase_json["m_Guid"] = phase.guid;
        phase_json["m_VehicleGuid"] = phase.vehicle_guid;
        phase_json["m_TimeDelaySeconds"] = phase.time_delay;
        phase_json["m_UndoGuid"] = nullptr;
        j["m_VehicleRestartPhases"].push_back(phase_json);
    }

    j["m_FlyingObjects"] = json::array();
    for (const auto &object : layout.flyingObjects) {
        json object_json;
        object_json["m_Pos"]["x"] = object.pos.x;
        object_json["m_Pos"]["y"] = object.pos.y;
        object_json["m_Pos"]["z"] = object.pos.z;
        object_json["m_Scale"]["x"] = object.scale.x;
        object_json["m_Scale"]["y"] = object.scale.y;
        object_json["m_Scale"]["z"] = object.scale.z;
        object_json["m_PrefabName"] = object.prefab_name;
        object_json["m_UndoGuid"] = nullptr;
        j["m_FlyingObjects"].push_back(object_json);
    }

    j["m_Rocks"] = json::array();
    for (const auto &rock : layout.rocks) {
        json rock_json;
        rock_json["m_Pos"]["x"] = rock.pos.x;
        rock_json["m_Pos"]["y"] = rock.pos.y;
        rock_json["m_Pos"]["z"] = rock.pos.z;
        rock_json["m_Scale"]["x"] = rock.scale.x;
        rock_json["m_Scale"]["y"] = rock.scale.y;
        rock_json["m_Scale"]["z"] = rock.scale.z;
        rock_json["m_PrefabName"] = rock.prefab_name;
        rock_json["m_Flipped"] = rock.flipped;
        rock_json["m_UndoGuid"] = nullptr;
        j["m_Rocks"].push_back(rock_json);
    }

    j["m_SupportPillars"] = json::array();
    for (const auto &pillar : layout.supportPillars) {
        json pillar_json;
        pillar_json["m_Pos"]["x"] = pillar.pos.x;
        pillar_json["m_Pos"]["y"] = pillar.pos.y;
        pillar_json["m_Pos"]["z"] = pillar.pos.z;
        pillar_json["m_Scale"]["x"] = pillar.scale.x;
        pillar_json["m_Scale"]["y"] = pillar.scale.y;
        pillar_json["m_Scale"]["z"] = pillar.scale.z;
        pillar_json["m_PrefabName"] = pillar.prefab_name;
        pillar_json["m_UndoGuid"] = nullptr;
        j["m_SupportPillars"].push_back(pillar_json);
    }

    j["m_WaterBlocks"] = json::array();
    for (const auto &water : layout.waterBlocks) {
        json water_json;
        water_json["m_Pos"]["x"] = water.pos.x;
        water_json["m_Pos"]["y"] = water.pos.y;
        water_json["m_Pos"]["z"] = water.pos.z;
        water_json["m_Width"] = water.width;
        water_json["m_Height"] = water.height;
        water_json["m_LockPosition"] = water.lock_position;
        water_json["m_UndoGuid"] = nullptr;
        j["m_WaterBlocks"].push_back(water_json);
    }

    j["m_CustomShapes"] = json::array();
    for (const auto &shape : layout.customShapes) {
        json shape_json;
        shape_json["m_Pos"]["x"] = shape.pos.x;
        shape_json["m_Pos"]["y"] = shape.pos.y;
        shape_json["m_Pos"]["z"] = shape.pos.z;

        shape_json["m_Scale"]["x"] = shape.scale.x;
        shape_json["m_Scale"]["y"] = shape.scale.y;
        shape_json["m_Scale"]["z"] = shape.scale.z;

        shape_json["m_Rot"]["x"] = shape.rot.x;
        shape_json["m_Rot"]["y"] = shape.rot.y;
        shape_json["m_Rot"]["z"] = shape.rot.z;
        shape_json["m_Rot"]["w"] = shape.rot.w;

        shape_json["m_Color"]["r"] = shape.color.r;
        shape_json["m_Color"]["g"] = shape.color.g;
        shape_json["m_Color"]["b"] = shape.color.b;
        shape_json["m_Color"]["a"] = shape.color.a;

        shape_json["m_Flipped"] = shape.flipped;
        shape_json["m_CollidesWithRoad"] = shape.collides_with_road;
        shape_json["m_CollidesWithNodes"] = shape.collides_with_nodes;
        shape_json["m_CollidesWithSplitNodes"] = shape.collides_with_split_nodes;
        shape_json["m_Dynamic"] = shape.dynamic;
        shape_json["m_RotationDegrees"] = shape.rotation_degrees;
        shape_json["m_Mass"] = shape.mass;
        shape_json["m_Bounciness"] = shape.bounciness;
        shape_json["m_PinMotorStrength"] = shape.pin_motor_strength;
        shape_json["m_PinTargetVelocity"] = shape.pin_target_velocity;

        shape_json["m_PointsLocalSpace"] = json::array();
        for (const Vec2 &point : shape.points_local_space) {
            json point_json;
            point_json["x"] = point.x;
            point_json["y"] = point.y;
            shape_json["m_PointsLocalSpace"].push_back(point_json);
        }

        shape_json["m_StaticPins"] = json::array();
        for (const Vec3 &point : shape.static_pins) {
            json point_json;
            point_json["x"] = point.x;
            point_json["y"] = point.y;
            point_json["z"] = point.z;
            shape_json["m_StaticPins"].push_back(point_json);
        }

        shape_json["m_DynamicAnchorGuids"] = json::array();
        for (const std::string &guid : shape.dynamic_anchor_guids) {
            shape_json["m_DynamicAnchorGuids"].push_back(guid);
        }

        shape_json["m_UndoGuid"] = nullptr;
        j["m_CustomShapes"].push_back(shape_json);
    }
    auto budget = j["m_Budget"];
    budget["m_CashBudget"] = layout.budget.cash;
    budget["m_RoadBudget"] = layout.budget.road;
    budget["m_WoodBudget"] = layout.budget.wood;
    budget["m_SteelBudget"] = layout.budget.steel;
    budget["m_HydraulicBudget"] = layout.budget.hydraulics;
    budget["m_RopeBudget"] = layout.budget.rope;
    budget["m_CableBudget"] = layout.budget.cable;
    budget["m_SpringBudget"] = layout.budget.spring;
    budget["m_BungieRopeBudget"] = layout.budget.bungee_rope;
    budget["m_AllowWood"] = layout.budget.allow_wood;
    budget["m_AllowSteel"] = layout.budget.allow_steel;
    budget["m_AllowHydraulic"] = layout.budget.allow_hydraulics;
    budget["m_AllowRope"] = layout.budget.allow_rope;
    budget["m_AllowCable"] = layout.budget.allow_cable;
    budget["m_AllowSpring"] = layout.budget.allow_spring;
    budget["m_AllowReinforcedRoad"] = layout.budget.allow_reinforced_road;
    j["m_Budget"] = budget;

    j["m_Settings"]["m_HydraulicControllerEnabled"] = layout.settings.hydraulics_controller_enabled;
    j["m_Settings"]["m_Unbreakable"] = layout.settings.unbreakable;

    auto workshop = j["m_Workshop"];
    workshop["m_Id"] = layout.workshop.id;
    workshop["m_LeaderboardId"] = layout.workshop.leaderboard_id;
    workshop["m_Title"] = layout.workshop.title;
    workshop["m_Description"] = layout.workshop.description;
    workshop["m_AutoPlay"] = layout.workshop.autoplay;
    workshop["m_Tags"] = layout.workshop.tags;

    j["m_Workshop"] = workshop;

    // Mod support
    j["ext_Mods"] = json::array();
    for (auto &m : layout.modData.mods) {
        json mod;
        mod["name"] = m.name;
        mod["version"] = m.version;
        mod["settings"] = m.settings;
        j["ext_Mods"].push_back(mod);
    }
    for (auto &md : layout.modData.mod_save_data) {
        json mod;
        mod["name"] = md.name;
        mod["version"] = md.version;
        if (md.data != nullptr) {
            mod["base64_encoded_data"] = macaron::Base64::Encode(md.data);
        } else {
            mod["base64_encoded_data"] = "";
        }
        j["ext_ModSaveData"].push_back(mod);
    }

    std::ofstream of(path, std::ios::out);
    of << j.dump(2);
    of.close();
}

Layout load_json(std::string &json_str) {
    Layout layout;
    json j = json::parse(json_str);
    layout.version = j["m_Version"].get<int>();

    // Bridge
    auto b = j["m_Bridge"];

    // Bridge anchors
    for (auto &a : b["m_Anchors"]) {
        BridgeJoint anchor;
        anchor.pos.x = a["m_Pos"]["x"].get<float>();
        anchor.pos.y = a["m_Pos"]["y"].get<float>();
        anchor.pos.z = a["m_Pos"]["z"].get<float>();
        anchor.is_anchor = a["m_IsAnchor"].get<bool>();
        anchor.is_split = a["m_IsSplit"].get<bool>();
        anchor.guid = a["m_Guid"].get<std::string>();
        layout.bridge.anchors.push_back(anchor);
    }

    // Anchors
    for (auto &a : j["m_Anchors"]) {
        BridgeJoint anchor;
        anchor.pos.x = a["m_Pos"]["x"].get<float>();
        anchor.pos.y = a["m_Pos"]["y"].get<float>();
        anchor.pos.z = a["m_Pos"]["z"].get<float>();
        anchor.is_anchor = a["m_IsAnchor"].get<bool>();
        anchor.is_split = a["m_IsSplit"].get<bool>();
        anchor.guid = a["m_Guid"].get<std::string>();
        layout.anchors.push_back(anchor);
    }

    // Bridge edges
    for (auto &e : b["m_BridgeEdges"]) {
        BridgeEdge edge;
        edge.joint_a_part = (SplitJointPart)e["m_JointAPart"].get<int>();
        edge.joint_b_part = (SplitJointPart)e["m_JointBPart"].get<int>();
        edge.material_type = (BridgeMaterialType)e["m_Material"].get<int>();
        edge.node_a_guid = e["m_NodeA_Guid"].get<std::string>();
        edge.node_b_guid = e["m_NodeB_Guid"].get<std::string>();
        layout.bridge.edges.push_back(edge);
    }

    // Bridge joints
    for (auto &jo : b["m_BridgeJoints"]) {
        BridgeJoint joint;
        joint.guid = jo["m_Guid"].get<std::string>();
        joint.pos.x = jo["m_Pos"]["x"].get<float>();
        joint.pos.y = jo["m_Pos"]["y"].get<float>();
        joint.pos.z = jo["m_Pos"]["z"].get<float>();
        joint.is_anchor = jo["m_IsAnchor"].get<bool>();
        joint.is_split = jo["m_IsSplit"].get<bool>();
        layout.bridge.joints.push_back(joint);
    }

    // Bridge springs
    for (auto &s : b["m_BridgeSprings"]) {
        BridgeSpring spring;
        spring.guid = s["m_Guid"].get<std::string>();
        spring.node_a_guid = s["m_NodeA_Guid"].get<std::string>();
        spring.node_b_guid = s["m_NodeB_Guid"].get<std::string>();
        spring.normalized_value = s["m_NormalizedValue"].get<float>();
        layout.bridge.springs.push_back(spring);
    }

    // Hydraulic controller
    for (auto &p : b["m_HydraulicsController"]["m_Phases"]) {
        HydraulicsControllerPhase phase;
        for (auto &sj : p["m_BridgeSplitJoints"]) {
            BridgeSplitJoint split_joint;
            split_joint.guid = sj["m_BridgeJointGuid"].get<std::string>();
            split_joint.state = (SplitJointState)sj["m_SplitJointState"].get<int>();
            phase.bridge_split_joints.push_back(split_joint);
        }
        phase.hydraulics_phase_guid = p["m_HydraulicsPhaseGuid"].get<std::string>();
        for (auto &pg : p["m_PistonGuids"]) {
            phase.piston_guids.push_back(pg.get<std::string>());
        }
        phase.disable_new_additions = p["m_DisableNewAdditions"];
        layout.bridge.phases.push_back(phase);
    }

    // Bridge pistons
    for (auto &ps : b["m_Pistons"]) {
        Piston piston;
        piston.guid = ps["m_Guid"].get<std::string>();
        piston.node_a_guid = ps["m_NodeA_Guid"].get<std::string>();
        piston.node_b_guid = ps["m_NodeB_Guid"].get<std::string>();
        piston.normalized_value = ps["m_NormalizedValue"].get<float>();
        layout.bridge.pistons.push_back(piston);
    }

    // Bridge version
    layout.bridge.version = b["m_Version"].get<int>();

    // Budget
    auto budget = j["m_Budget"];
    Budget budget_data{};
    budget_data.allow_cable = budget["m_AllowCable"].get<bool>();
    budget_data.allow_hydraulics = budget["m_AllowHydraulic"].get<bool>();
    budget_data.allow_reinforced_road = budget["m_AllowReinforcedRoad"].get<bool>();
    budget_data.allow_rope = budget["m_AllowRope"].get<bool>();
    budget_data.allow_spring = budget["m_AllowSpring"].get<bool>();
    budget_data.allow_steel = budget["m_AllowSteel"].get<bool>();
    budget_data.allow_wood = budget["m_AllowWood"].get<bool>();
    budget_data.bungee_rope = budget["m_BungieRopeBudget"].get<int>();  // it's spelled like this in the code
    budget_data.cable = budget["m_CableBudget"].get<int>();
    budget_data.cash = budget["m_CashBudget"].get<int>();
    budget_data.hydraulics = budget["m_HydraulicBudget"].get<int>();
    budget_data.road = budget["m_RoadBudget"].get<int>();
    budget_data.rope = budget["m_RopeBudget"].get<int>();
    budget_data.spring = budget["m_SpringBudget"].get<int>();
    budget_data.steel = budget["m_SteelBudget"].get<int>();
    budget_data.wood = budget["m_WoodBudget"].get<int>();
    layout.budget = budget_data;

    // Event timelines
    for (auto &t : j["m_EventTimelines"]) {
        EventTimeline timeline;
        timeline.checkpoint_guid = t["m_CheckpointGuid"].get<std::string>();
        // Stages
        for (auto &s : t["m_Stages"]) {
            EventStage stage;
            // Units
            for (auto &u : s["m_Units"]) {
                EventUnit unit;
                unit.guid = u["m_Guid"].get<std::string>();
                stage.units.push_back(unit);
            }
            timeline.stages.push_back(stage);
        }
        layout.eventTimelines.push_back(timeline);
    }

    // Hydraulic phases
    for (auto &h : j["m_HydraulicPhases"]) {
        HydraulicPhase phase;
        phase.guid = h["m_Guid"].get<std::string>();
        phase.time_delay = h["m_TimeDelaySeconds"].get<float>();
        layout.phases.push_back(phase);
    }

    // Layout settings
    auto settings = j["m_Settings"];
    layout.settings.hydraulics_controller_enabled = settings["m_HydraulicControllerEnabled"].get<bool>();
    layout.settings.unbreakable = settings["m_Unbreakable"].get<bool>();

    // Terrain stretches
    for (auto &ts : j["m_TerrainStretches"]) {
        TerrainIsland stretch;
        stretch.flipped = ts["m_Flipped"].get<bool>();
        stretch.height_added = ts["m_HeightAdded"].get<float>();
        stretch.lock_position = ts["m_LockPosition"].get<bool>();
        stretch.pos.x = ts["m_Pos"]["x"].get<float>();
        stretch.pos.y = ts["m_Pos"]["y"].get<float>();
        stretch.pos.z = ts["m_Pos"]["z"].get<float>();
        stretch.prefab_name = ts["m_PrefabName"].get<std::string>();
        stretch.right_edge_water_height = ts["m_RightEdgeWaterHeight"].get<float>();
        stretch.terrain_island_type = (TerrainIslandType)ts["m_TerrainIslandType"].get<int>();
        stretch.variant_index = ts["m_VariantIndex"].get<int>();
        layout.terrainStretches.push_back(stretch);
    }

    // Theme stub key
    layout.stubKey = j["m_ThemeStubKey"].get<std::string>();

    // Water blocks
    for (auto &wb : j["m_WaterBlocks"]) {
        WaterBlock water;
        water.height = wb["m_Height"].get<float>();
        water.lock_position = wb["m_LockPosition"].get<bool>();
        water.pos.x = wb["m_Pos"]["x"].get<float>();
        water.pos.y = wb["m_Pos"]["y"].get<float>();
        water.pos.z = wb["m_Pos"]["z"].get<float>();
        water.width = wb["m_Width"].get<float>();

        layout.waterBlocks.push_back(water);
    }

    // Workshop data
    auto workshop = j["m_Workshop"];
    layout.workshop.autoplay = workshop["m_AutoPlay"].get<bool>();
    layout.workshop.description = workshop["m_Description"].get<std::string>();
    layout.workshop.id = workshop["m_Id"].get<std::string>();
    layout.workshop.leaderboard_id = workshop["m_LeaderboardId"].get<std::string>();
    // tags
    for (auto &t : workshop["m_Tags"]) {
        layout.workshop.tags.push_back(t.get<std::string>());
    }
    layout.workshop.title = workshop["m_Title"].get<std::string>();

    // Z-axis vehicles
    for (auto &zv : j["m_ZedAxisVehicles"]) {
        ZAxisVehicle z;
        z.guid = zv["m_Guid"].get<std::string>();
        z.pos.x = zv["m_Pos"]["x"].get<float>();
        z.pos.y = zv["m_Pos"]["y"].get<float>();
        z.prefab_name = zv["m_PrefabName"].get<std::string>();
        z.speed = zv["m_Speed"].get<float>();
        z.time_delay = zv["m_TimeDelaySeconds"].get<float>();
        z.rot.x = zv["m_Rot"]["x"].get<float>();
        z.rot.y = zv["m_Rot"]["y"].get<float>();
        z.rot.z = zv["m_Rot"]["z"].get<float>();
        z.rot.w = zv["m_Rot"]["w"].get<float>();
        z.rotation_degrees = zv["m_RotationDegrees"].get<float>();
        layout.zAxisVehicles.push_back(z);
    }

    // Checkpoints
    for (auto &c : j["m_Checkpoints"]) {
        Checkpoint point;
        point.pos.x = c["m_Pos"]["x"].get<float>();
        point.pos.y = c["m_Pos"]["y"].get<float>();
        point.prefab_name = c["m_PrefabName"].get<std::string>();
        point.vehicle_guid = c["m_VehicleGuid"].get<std::string>();
        point.vehicle_restart_phase_guid = c["m_VehicleRestartPhaseGuid"].get<std::string>();
        point.trigger_timeline = c["m_TriggerTimeline"].get<bool>();
        point.stop_vehicle = c["m_StopVehicle"].get<bool>();
        point.reverse_vehicle_on_restart = c["m_ReverseVehicleOnRestart"].get<bool>();
        point.guid = c["m_Guid"].get<std::string>();
        layout.checkpoints.push_back(point);
    }

    // Custom Shapes
    for (auto &cs : j["m_CustomShapes"]) {
        CustomShape shape;
        shape.bounciness = cs["m_Bounciness"].get<float>();
        shape.collides_with_road = cs["m_CollidesWithRoad"].get<bool>();
        shape.collides_with_nodes = cs["m_CollidesWithNodes"].get<bool>();
        shape.collides_with_split_nodes = cs["m_CollidesWithSplitNodes"].get<bool>();
        shape.color.r = cs["m_Color"]["r"].get<float>();
        shape.color.g = cs["m_Color"]["g"].get<float>();
        shape.color.b = cs["m_Color"]["b"].get<float>();
        shape.color.a = cs["m_Color"]["a"].get<float>();
        shape.dynamic = cs["m_Dynamic"].get<bool>();
        shape.flipped = cs["m_Flipped"].get<bool>();
        shape.mass = cs["m_Mass"].get<float>();
        shape.pin_motor_strength = cs["m_PinMotorStrength"].get<float>();
        shape.pin_target_velocity = cs["m_PinTargetVelocity"].get<float>();
        // points
        for (auto &p : cs["m_PointsLocalSpace"]) {
            shape.points_local_space.push_back(Vec2{p["x"].get<float>(), p["y"].get<float>()});
        }
        shape.pos.x = cs["m_Pos"]["x"].get<float>();
        shape.pos.y = cs["m_Pos"]["y"].get<float>();
        shape.pos.z = cs["m_Pos"]["z"].get<float>();

        shape.rot.x = cs["m_Rot"]["x"].get<float>();
        shape.rot.y = cs["m_Rot"]["y"].get<float>();
        shape.rot.z = cs["m_Rot"]["z"].get<float>();
        shape.rot.w = cs["m_Rot"]["w"].get<float>();

        shape.scale.x = cs["m_Scale"]["x"].get<float>();
        shape.scale.y = cs["m_Scale"]["y"].get<float>();
        shape.scale.z = cs["m_Scale"]["z"].get<float>();

        shape.rotation_degrees = cs["m_RotationDegrees"].get<float>();

        // static pins
        for (auto &p : cs["m_StaticPins"]) {
            shape.static_pins.push_back(Vec3{p["x"].get<float>(), p["y"].get<float>(), p["z"].get<float>()});
        }

        // dynamic anchor GUIDs
        for (auto &g : cs["m_DynamicAnchorGuids"]) {
            shape.dynamic_anchor_guids.push_back(g.get<std::string>());
        }

        layout.customShapes.push_back(shape);
    }

    // Flying Objects
    for (auto &fo : j["m_FlyingObjects"]) {
        FlyingObject object;
        object.pos.x = fo["m_Pos"]["x"].get<float>();
        object.pos.y = fo["m_Pos"]["y"].get<float>();
        object.pos.z = fo["m_Pos"]["z"].get<float>();

        object.scale.x = fo["m_Scale"]["x"].get<float>();
        object.scale.y = fo["m_Scale"]["y"].get<float>();
        object.scale.z = fo["m_Scale"]["z"].get<float>();

        object.prefab_name = fo["m_PrefabName"].get<std::string>();

        layout.flyingObjects.push_back(object);
    }

    // Pillars
    for (auto &pl : j["m_Pillars"]) {
        Pillar pillar;
        pillar.height = pl["m_Height"].get<float>();

        pillar.pos.x = pl["m_Pos"]["x"].get<float>();
        pillar.pos.y = pl["m_Pos"]["y"].get<float>();
        pillar.pos.z = pl["m_Pos"]["z"].get<float>();

        pillar.prefab_name = pl["m_PrefabName"].get<std::string>();

        layout.pillars.push_back(pillar);
    }

    // Support pillars
    for (auto &sp : j["m_SupportPillars"]) {
        SupportPillar pillar;

        pillar.pos.x = sp["m_Pos"]["x"].get<float>();
        pillar.pos.y = sp["m_Pos"]["y"].get<float>();
        pillar.pos.z = sp["m_Pos"]["z"].get<float>();

        pillar.scale.x = sp["m_Scale"]["x"].get<float>();
        pillar.scale.y = sp["m_Scale"]["y"].get<float>();
        pillar.scale.z = sp["m_Scale"]["z"].get<float>();

        pillar.prefab_name = sp["m_PrefabName"].get<std::string>();

        layout.supportPillars.push_back(pillar);
    }

    // Vehicle restart phases
    for (auto &vr : j["m_VehicleRestartPhases"]) {
        VehicleRestartPhase phase;
        phase.guid = vr["m_Guid"].get<std::string>();
        phase.time_delay = vr["m_TimeDelaySeconds"].get<float>();
        phase.vehicle_guid = vr["m_VehicleGuid"].get<std::string>();

        layout.vehicleRestartPhases.push_back(phase);
    }

    // Vehicle stop triggers
    for (auto &st : j["m_VehicleStopTriggers"]) {
        VehicleStopTrigger trigger;
        trigger.flipped = st["m_Flipped"].get<bool>();
        trigger.height = st["m_Height"].get<float>();

        trigger.pos.x = st["m_Pos"]["x"].get<float>();
        trigger.pos.y = st["m_Pos"]["y"].get<float>();

        trigger.rot.x = st["m_Rot"]["x"].get<float>();
        trigger.rot.y = st["m_Rot"]["y"].get<float>();
        trigger.rot.z = st["m_Rot"]["z"].get<float>();
        trigger.rot.w = st["m_Rot"]["w"].get<float>();

        trigger.prefab_name = st["m_PrefabName"].get<std::string>();

        trigger.rotation_degrees = st["m_RotationDegrees"].get<float>();
        trigger.stop_vehicle_guid = st["m_StopVehicleGuid"].get<std::string>();

        layout.vehicleStopTriggers.push_back(trigger);
    }

    // Vehicles
    for (auto &v : j["m_Vehicles"]) {
        Vehicle vh;
        vh.acceleration = v["m_Acceleration"].get<float>();
        vh.braking_force_multiplier = v["m_BrakingForceMultiplier"].get<float>();
        // checkpoint guids
        for (auto &g : v["m_CheckpointGuids"]) {
            vh.checkpoint_guids.push_back(g.get<std::string>());
        }
        vh.desired_acceleration = v["m_DesiredAcceleration"].get<float>();
        vh.display_name = v["m_DisplayName"].get<std::string>();
        vh.flipped = v["m_Flipped"].get<bool>();
        vh.guid = v["m_Guid"].get<std::string>();
        vh.idle_on_downhill = v["m_IdleOnDownhill"].get<bool>();
        vh.mass = v["m_Mass"].get<float>();
        vh.max_slope = v["m_MaxSlope"].get<float>();
        vh.ordered_checkpoints = v["m_OrderedCheckpoints"].get<bool>();

        vh.pos.x = v["m_Pos"]["x"].get<float>();
        vh.pos.y = v["m_Pos"]["y"].get<float>();

        vh.prefab_name = v["m_PrefabName"].get<std::string>();

        vh.rot.x = v["m_Rot"]["x"].get<float>();
        vh.rot.y = v["m_Rot"]["y"].get<float>();
        vh.rot.z = v["m_Rot"]["z"].get<float>();
        vh.rot.w = v["m_Rot"]["w"].get<float>();

        vh.rotation_degrees = v["m_RotationDegrees"].get<float>();
        vh.strength_method = (StrengthMethod)v["m_StrengthMethod"].get<int>();
        vh.time_delay = v["m_TimeDelaySeconds"].get<float>();
        layout.vehicles.push_back(vh);
    }

    // Platforms
    for (auto &p : j["m_Platforms"]) {
        Platform platform{};
        platform.flipped = p["m_Flipped"].get<bool>();
        platform.height = p["m_Height"].get<float>();
        platform.pos.x = p["m_Pos"]["x"].get<float>();
        platform.pos.y = p["m_Pos"]["y"].get<float>();
        platform.solid = p["m_Solid"].get<bool>();
        platform.width = p["m_Width"].get<float>();
        layout.platforms.push_back(platform);
    }

    // Ramps
    for (auto &r : j["m_Ramps"]) {
        Ramp ramp{};
        ramp.flipped_vertical = r["m_FlippedVertical"].get<bool>();
        ramp.flipped_horizontal = r["m_FlippedHorizontal"].get<bool>();
        ramp.flipped_legs = r["m_FlippedLegs"].get<bool>();
        ramp.height = r["m_Height"].get<float>();
        ramp.hide_legs = r["m_HideLegs"].get<bool>();
        ramp.num_segments = r["m_NumSegments"].get<int>();
        ramp.spline_type = (SplineType)r["m_SplineType"].get<int>();

        ramp.pos.x = r["m_Pos"]["x"].get<float>();
        ramp.pos.y = r["m_Pos"]["y"].get<float>();

        // Line points
        for (auto &p : r["m_LinePoints"]) {
            ramp.line_points.push_back(Vec2{p["x"].get<float>(), p["y"].get<float>()});
        }

        // Control points
        for (auto &p : r["m_ControlPoints"]) {
            ramp.control_points.push_back(Vec2{p["x"].get<float>(), p["y"].get<float>()});
        }

        layout.ramps.push_back(ramp);
    }

    // Finally, rocks
    for (auto &r : j["m_Rocks"]) {
        Rock rock{};
        rock.flipped = r["m_Flipped"].get<bool>();

        rock.pos.x = r["m_Pos"]["x"].get<float>();
        rock.pos.y = r["m_Pos"]["y"].get<float>();
        rock.pos.z = r["m_Pos"]["z"].get<float>();

        rock.prefab_name = r["m_PrefabName"].get<std::string>();

        rock.scale.x = r["m_Scale"]["x"].get<float>();
        rock.scale.y = r["m_Scale"]["y"].get<float>();
        rock.scale.z = r["m_Scale"]["z"].get<float>();

        layout.rocks.push_back(rock);
    }

    return layout;
}

void dump_slot_json(const SaveSlot& slot, const std::string& path) {
    json j;
    j["m_Version"] = slot.version;
    j["m_PhysicsVersion"] = slot.physicsVersion;
    j["m_SlotID"] = slot.slotId;
    j["m_DisplayName"] = slot.displayName;
    j["m_SlotFileName"] = slot.fileName;
    j["m_Budget"] = slot.budget;
    j["m_LastWriteTimeTicks"] = slot.lastWriteTimeTicks;
    auto b = nlohmann::json::object();
    b["m_Version"] = slot.bridge.version;
    b["m_BridgeJoints"] = nlohmann::json::array();
    for (const BridgeJoint& jnt : slot.bridge.joints) {
        auto joint = nlohmann::json::object();
        joint["m_Pos"]["x"] = jnt.pos.x;
        joint["m_Pos"]["y"] = jnt.pos.y;
        joint["m_Pos"]["z"] = jnt.pos.z;

        joint["m_IsAnchor"] = jnt.is_anchor;
        joint["m_IsSplit"] = jnt.is_split;

        joint["m_Guid"] = jnt.guid;
        b["m_BridgeJoints"].push_back(joint);
    }
    b["m_BridgeEdges"] = nlohmann::json::array();
    for (const BridgeEdge& e : slot.bridge.edges) {
        auto edge = nlohmann::json::object();
        edge["m_MaterialType"] = e.material_type;
        edge["m_NodeA_Guid"] = e.node_a_guid;
        edge["m_NodeB_Guid"] = e.node_b_guid;
        edge["m_JointAPart"] = e.joint_a_part;
        edge["m_JointBPart"] = e.joint_b_part;

        b["m_BridgeEdges"].push_back(edge);
    }
    b["m_BridgeSprings"] = nlohmann::json::array();
    for (const BridgeSpring& s : slot.bridge.springs) {
        auto spring = nlohmann::json::object();
        spring["m_NormalizedValue"] = s.normalized_value;
        spring["m_NodeA_Guid"] = s.node_a_guid;
        spring["m_NodeB_Guid"] = s.node_b_guid;
        spring["m_Guid"] = s.guid;

        b["m_BridgeSprings"].push_back(spring);
    }
    b["m_Pistons"] = nlohmann::json::array();
    for (const Piston& p : slot.bridge.pistons) {
        auto piston = nlohmann::json::object();
        piston["m_NormalizedValue"] = p.normalized_value;
        piston["m_NodeA_Guid"] = p.node_a_guid;
        piston["m_NodeB_Guid"] = p.node_b_guid;
        piston["m_Guid"] = p.guid;

        b["m_Pistons"].push_back(piston);
    }
    b["m_Anchors"] = nlohmann::json::array();
    for (const BridgeJoint& a : slot.bridge.anchors) {
        auto anchor = nlohmann::json::object();
        anchor["m_Pos"]["x"] = a.pos.x;
        anchor["m_Pos"]["y"] = a.pos.y;
        anchor["m_Pos"]["z"] = a.pos.z;

        anchor["m_IsAnchor"] = a.is_anchor;
        anchor["m_IsSplit"] = a.is_split;

        anchor["m_Guid"] = a.guid;
        b["m_Anchors"].push_back(anchor);
    }
    b["m_HydraulicsController"]["m_Phases"] = nlohmann::json::array();
    for (const HydraulicsControllerPhase& p : slot.bridge.phases) {
        auto phase = nlohmann::json::object();
        phase["m_HydraulicsPhaseGuid"] = p.hydraulics_phase_guid;
        phase["m_PistonGuids"] = nlohmann::json::array();
        for (const std::string& g : p.piston_guids) {
            phase["m_PistonGuids"].push_back(g);
        }
        phase["m_BridgeSplitJoints"] = nlohmann::json::array();
        for (const BridgeSplitJoint& jobj : p.bridge_split_joints) {
            auto sj = nlohmann::json::object();
            sj["m_BridgeJointGuid"] = jobj.guid;
            sj["m_SplitJointState"] = jobj.state;
        }
        b["m_HydraulicsController"]["m_Phases"].push_back(phase);
    }
    j["m_Bridge"] = b;

    j["m_UsingUnlimitedMaterials"] = slot.unlimitedMaterials;
    j["m_UsingUnlimitedBudget"] = slot.unlimitedBudget;

    std::ofstream of(path, std::ios::out);
    of << j.dump(2);
    of.close();
}


int main(int argc, char **argv) {
    const std::string help_msg = R"END(
    Usage:
        %s [-h] [-s | --silent] [-v | --verbose] [-o | --output <path>] [-t | --type (json|yaml)] <path>
    Options:
        -h, --help              Show this help message and exit.
        -s, --silent            Don't print anything to stdout.
        -o, --output <path>     Define the output path, otherwise will be <path>.json or <path>.layout.
        -t, --type <type>       The type of the output. Either JSON or YAML. Not yet implemented, defaults to JSON.
    Notes:
        Files formats are based on the extension. If you feed the converter a file with the extension .layout that is
        not a layout file, it will cause issues.

    )END";

    if (argc < 2) {
        printf(help_msg.c_str(), argv[0]);
        return 1;
    }

    int c;
    bool custom_path = false;
    std::string output_path;
    while ((c = getopt(argc, argv, "hso:")) != -1) {
        switch (c) {
            case 'h':
                printf(help_msg.c_str(), argv[0]);
                return 0;
            case 's':
                silent = true;
                break;
            case 'o':
                custom_path = true;
                output_path = optarg;

                // check if the custom path directory exists
                if (!U::directory_of_file_exists(output_path)) {
                    U::log_error("Directory of output path does not exist.");
                    return 1;
                }

                break;
            default:
                break;
        }
    }

    std::string path = argv[argc - 1];
    // first, check if the file is a json file
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cout << "[\x1B[ Could not open file " << path << std::endl;
        return 1;
    }

    auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());

    if (path.ends_with(".layout.json")) {
        std::string json;
        std::ifstream fs(path);
        std::string line;
        while (std::getline(fs, line)) {
            json += line;
        }
        fs.close();

        U::log_info("Parsing JSON file...");
        Layout layout = load_json(json);

        if (custom_path) {
            path = output_path;
        } else {
            path += ".layout";
        }

        Serializer serializer(path, layout);
        serializer.serializeLayout();
        Utils::log_info("Layout serialized to " + path);
    } else if (path.ends_with(".layout")) {
        Deserializer deserializer(path);
        Layout layout = deserializer.deserializeLayout();

        if (custom_path) {
            path = output_path;
        } else {
            path += ".json";
        }

        dump_json(layout, path);
        Utils::log_info("Wrote JSON to " + path);
    } else if (path.ends_with(".slot")) {
        SlotDeserializer deserializer(path);
        SaveSlot slot = deserializer.deserializeSlot();

         if (custom_path) {
            path = output_path;
         } else {
             path += ".json";
         }

        dump_slot_json(slot, path);
        Utils::log_info("Wrote JSON to " + path);
    } else if (path.ends_with(".slot.json")) {
        U::log_info_d("Slot JSON files are not yet supported.");
    } else {
        U::log_error("File format not supported.");
        return 1;
    }

    std::cout << "\n";

    auto end = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
    Utils::log_info_d("Done! (" + std::to_string((double)(end - start).count() / 1000000.0) + "ms)");
    return 0;
}
