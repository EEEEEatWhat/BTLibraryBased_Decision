/**
 * @author suzukisuncy
 * @date 2023/11/27
 * @brief 通过映射表来解析数据,映射表接受cmd_id，对应DJI裁判系统附录,基于DJI裁判系统协议V1.6
*/

#pragma once 
// #include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <mutex>
#include "enums.h"
namespace RM_referee{
    // Packet header structure
    #pragma pack(1)
    struct PacketHeader {
        uint8_t SOF;
        uint16_t DataLength;
        uint8_t SequenceNumber;
        uint8_t CRC8;
    } ;
    #pragma pack()
    static_assert(sizeof(PacketHeader) == 5, "PacketHeader must be 5 bytes long with packing");
    static constexpr uint8_t StartOfFrame = 0xa5;

    // Base packet
    class RefereePacket {
        public:
            RefereePacket(){};
            virtual ~RefereePacket(){};
            virtual uint16_t GetID() = 0;
            virtual uint16_t GetDataLength() = 0;
            /**
             * @return 处理的字节数
            */
            virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) {
                (void)cmd_id;
                (void)data;
                (void)data_size;
                std::cout<<"[Warning : This should not appear !]\n";
                return 0 ;
            };

    };
    
    #define GENERATEPACK(TYPE,STRUCT) \
    class TYPE##Packet : public RefereePacket { \
    protected:\
    public:\
        STRUCT m_value;\
        std::mutex m_mutex;\
        TYPE##Packet(){};\
        ~TYPE##Packet(){};\
        void testsuccess(){std::cout<<"\nSuccess!!!\n";};\
        static uint16_t StaticGetID(){return uint16_t(PacketType::TYPE);};\
        uint16_t GetID(){return StaticGetID();};\
        static uint16_t StaticGetDataLength(){return sizeof(STRUCT);};\
        uint16_t GetDataLength() {return StaticGetDataLength();};\
        virtual uint16_t SolvePacket(uint16_t cmd_id, uint8_t* data ,uint16_t data_size) override ; \
    };

    /**
    @brief  如何使用：
            How to use GENERATEPACK(TYPE,STRUCT):
            Example：0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            定义数据包结构体并断言数据包大小
            
            struct ExtSupplyProjectileActionStruct { 
                uint8_t reserved; 
                uint8_t supply_robot_id;  
                uint8_t supply_projectile_step; 
                uint8_t supply_projectile_num; 
            };
            static_assert(sizeof(ExtSupplyProjectileActionStruct) == 4, "ExtSupplyProjectileActionStruct must be 4 bytes long with packing");
            GENERATEPACK(ExtSupplyProjectileAction,ExtSupplyProjectileActionStruct)
    @warning  type,struct不要重复；type请查阅enum.h
    @brief 等效于：
            //0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
            class ExtSupplyProjectileActionPacket : public RefereePacket { 
            TODO fix it
            };
    */


    //0x0001 GameStatusPacket 11 GameStatus
    struct GameStatusStruct { 
        uint8_t game_type : 4; 
        uint8_t game_progress : 4; 
        uint16_t stage_remain_time; 
        uint64_t SyncTimeStamp; }; 
    // static_assert(sizeof(GameStatusStruct) == 4, "GameStatusStruct must be 4 bytes long with packing");
    // GENERATEPACK(GameStatus,GameStatusStruct)

    //0x0002 GameResultEventPacket 1 GameResultEvent
    struct GameResultEventStruct { 
        uint8_t winner; }; 
    // static_assert(sizeof(GameResultEventStruct) == 4, "GameResultEventStruct must be 4 bytes long with packing");
    GENERATEPACK(GameResultEvent,GameResultEventStruct)

    //0x0003 GameRobotHPPacket 32 GameRobotHP
    struct GameRobotHPStruct { 
        uint16_t red_1_robot_HP; 
        uint16_t red_2_robot_HP; 
        uint16_t red_3_robot_HP; 
        uint16_t red_4_robot_HP; 
        uint16_t red_5_robot_HP; 
        uint16_t red_7_robot_HP; 
        uint16_t red_outpost_HP; 
        uint16_t red_base_HP; 
        uint16_t blue_1_robot_HP; 
        uint16_t blue_2_robot_HP; 
        uint16_t blue_3_robot_HP; 
        uint16_t blue_4_robot_HP; 
        uint16_t blue_5_robot_HP; 
        uint16_t blue_7_robot_HP; 
        uint16_t blue_outpost_HP; 
        uint16_t blue_base_HP; };
    // static_assert(sizeof(GameRobotHPStruct) == 4, "GameRobotHPStruct must be 4 bytes long with packing");
    GENERATEPACK(GameRobotHP,GameRobotHPStruct)

    //0x0101  PlaygroundEventPacket 4 PlaygroundEvent
    struct PlaygroundEventStruct { 
        uint32_t event_data; };
    static_assert(sizeof(PlaygroundEventStruct) == 4, "PlaygroundEventStruct must be 4 bytes long with packing");
    GENERATEPACK(PlaygroundEvent,PlaygroundEventStruct)

    //0x0102 ExtSupplyProjectileActionPacket 4 ExtSupplyProjectileAction
    struct ExtSupplyProjectileActionStruct { 
        uint8_t reserved; 
        uint8_t supply_robot_id;  
        uint8_t supply_projectile_step; 
        uint8_t supply_projectile_num; };
    static_assert(sizeof(ExtSupplyProjectileActionStruct) == 4, "ExtSupplyProjectileActionStruct must be 4 bytes long with packing");
    GENERATEPACK(ExtSupplyProjectileAction,ExtSupplyProjectileActionStruct)

    //0x0104 2

    //0x0105 1
    
    //0x0201 27

    //0x0202 PowerHeatDataPacket 16 PowerHeatData
    struct PowerHeatDataStruct { 
        uint16_t chassis_voltage; 
        uint16_t chassis_current; 
        float chassis_power; 
        uint16_t buffer_energy; 
        uint16_t shooter_17mm_1_barrel_heat; 
        uint16_t shooter_17mm_2_barrel_heat; 
        uint16_t shooter_42mm_barrel_heat; }; 
    static_assert(sizeof(PowerHeatDataStruct) == 16, "PowerHeatDataStruct must be 16 bytes long with packing");
    GENERATEPACK(PowerHeatData,PowerHeatDataStruct)

    //0x0203 16
    //0x0204 1
    //0x0205 1
    //0x0206 1
    //0x0207 7
    //0x0208 6
    //0x0209 4
    //0x020A 6
    //0x020B 40
    //0x020C 6
    //0x020D 4
    
    //0x0301 128
    //0x0302 CustomRobotDataPacket 30 CustomRobotData
    struct CustomRobotDataStruct { 
        uint8_t data[30]; };
    static_assert(sizeof(CustomRobotDataStruct) == 30, "CustomRobotDataStruct must be 30 bytes long with packing");
    GENERATEPACK(CustomRobotData,CustomRobotDataStruct)
    //0x0303 15
    //0x0304 12 
    //0x0305 10
    //0x0306 8
    //0x0307 103
    //0x0308 34

}