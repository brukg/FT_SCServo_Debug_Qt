#ifndef HLSCL_H
#define HLSCL_H

//-------EPROM(read only)--------
#define HLSCL_MODEL_L 3
#define HLSCL_MODEL_H 4

//-------EPROM(read write)--------
#define HLSCL_ID 5
#define HLSCL_BAUD_RATE 6
#define HLSCL_SECOND_ID 7
#define HLSCL_MIN_ANGLE_LIMIT_L 9
#define HLSCL_MIN_ANGLE_LIMIT_H 10
#define HLSCL_MAX_ANGLE_LIMIT_L 11
#define HLSCL_MAX_ANGLE_LIMIT_H 12
#define HLSCL_CW_DEAD 26
#define HLSCL_CCW_DEAD 27
#define HLSCL_OFS_L 31
#define HLSCL_OFS_H 32
#define HLSCL_MODE 33

//-------SRAM(read write)--------
#define HLSCL_TORQUE_ENABLE 40
#define HLSCL_ACC 41
#define HLSCL_GOAL_POSITION_L 42
#define HLSCL_GOAL_POSITION_H 43
#define HLSCL_GOAL_TORQUE_L 44
#define HLSCL_GOAL_TORQUE_H 45
#define HLSCL_GOAL_SPEED_L 46
#define HLSCL_GOAL_SPEED_H 47
#define HLSCL_TORQUE_LIMIT_L 48
#define HLSCL_TORQUE_LIMIT_H 49
#define HLSCL_LOCK 55

//-------SRAM(read only)--------
#define HLSCL_PRESENT_POSITION_L 56
#define HLSCL_PRESENT_POSITION_H 57
#define HLSCL_PRESENT_SPEED_L 58
#define HLSCL_PRESENT_SPEED_H 59
#define HLSCL_PRESENT_LOAD_L 60
#define HLSCL_PRESENT_LOAD_H 61
#define HLSCL_PRESENT_VOLTAGE 62
#define HLSCL_PRESENT_TEMPERATURE 63
#define HLSCL_MOVING 66
#define HLSCL_PRESENT_CURRENT_L 69
#define HLSCL_PRESENT_CURRENT_H 70

#include <vector>

namespace feetech_servo
{

// HLS-series servos. Ported from FTServo_Linux/src/HLSCL.cpp.
//
// Unlike SMS/STS, addresses 44/45 hold Goal Torque -- the maximum torque
// current for the move, in units of 6.5mA. Writing 0 there commands a 0mA
// limit and the servo will not move. This is why driving an HLS servo through
// SMS_STS::write_pos_ex (which zero-fills 44/45) appears to do nothing.
class HLSCL
{
public:
    HLSCL(SCSerial *scserial) : scserial_(scserial) {}

	int write_pos_ex(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)
    {
        uint8_t bBuf[7];
        pack_goal(bBuf, Position, Speed, ACC, Torque);
        return scserial_->gen_write(ID, HLSCL_ACC, bBuf, 7);
    }

	int reg_write_pos_ex(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)
    {
        uint8_t bBuf[7];
        pack_goal(bBuf, Position, Speed, ACC, Torque);
        return scserial_->reg_write(ID, HLSCL_ACC, bBuf, 7);
    }

	void sync_write_pos_ex(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[], uint16_t Torque[])
    {
        std::vector<uint8_t> offbuf(7 * IDN);
        for(uint8_t i = 0; i<IDN; i++)
        {
            pack_goal(offbuf.data() + i*7,
                      Position[i],
                      Speed ? Speed[i] : 0,
                      ACC ? ACC[i] : 0,
                      Torque ? Torque[i] : 0);
        }
        scserial_->sync_write(ID, IDN, HLSCL_ACC, offbuf.data(), 7);
    }

	int write_spe(uint8_t ID, int16_t Speed, uint8_t ACC = 0, uint16_t Torque = 0)//恒速模式控制指令
    {
        if(Speed<0){
            Speed = -Speed;
            Speed |= (1<<15);
        }
        uint8_t bBuf[7];
        bBuf[0] = ACC;
        scserial_->host_2_scs(bBuf+1, bBuf+2, 0);
        scserial_->host_2_scs(bBuf+3, bBuf+4, Torque);
        scserial_->host_2_scs(bBuf+5, bBuf+6, Speed);

        return scserial_->gen_write(ID, HLSCL_ACC, bBuf, 7);
    }

	int write_ele(uint8_t ID, int16_t Torque)//恒力模式控制指令
    {
        if(Torque<0){
            Torque = -Torque;
            Torque |= (1<<15);
        }
        return scserial_->write_word(ID, HLSCL_GOAL_TORQUE_L, Torque);
    }

	int servo_mode(uint8_t ID)
    {
        return scserial_->write_byte(ID, HLSCL_MODE, 0);
    }

	int wheel_mode(uint8_t ID)//恒速模式
    {
        return scserial_->write_byte(ID, HLSCL_MODE, 1);
    }

	int ele_mode(uint8_t ID)//恒力模式
    {
        return scserial_->write_byte(ID, HLSCL_MODE, 2);
    }

	int enable_torque(uint8_t ID, uint8_t Enable)
    {
        return scserial_->write_byte(ID, HLSCL_TORQUE_ENABLE, Enable);
    }

	int unlock_eprom(uint8_t ID)
    {
        enable_torque(ID, 0);
        return scserial_->write_byte(ID, HLSCL_LOCK, 0);
    }

	int lock_eprom(uint8_t ID)
    {
        return scserial_->write_byte(ID, HLSCL_LOCK, 1);
    }

	int read_pos(int ID)
    {
        int Pos = scserial_->read_word(ID, HLSCL_PRESENT_POSITION_L);
        if(Pos == -1)
            return -1;
        if(Pos&(1<<15)){
            Pos = -(Pos&~(1<<15));
        }
        return Pos;
    }

	int read_speed(int ID)
    {
        int Speed = scserial_->read_word(ID, HLSCL_PRESENT_SPEED_L);
        if(Speed == -1)
            return -1;
        if(Speed&(1<<15)){
            Speed = -(Speed&~(1<<15));
        }
        return Speed;
    }

private:
    // Addresses 41..47 in one burst: ACC, Goal Position, Goal Torque, Goal Speed.
    void pack_goal(uint8_t *buf, int16_t Position, uint16_t Speed, uint8_t ACC, uint16_t Torque)
    {
        if(Position<0){
            Position = -Position;
            Position |= (1<<15);
        }
        buf[0] = ACC;
        scserial_->host_2_scs(buf+1, buf+2, Position);
        scserial_->host_2_scs(buf+3, buf+4, Torque);
        scserial_->host_2_scs(buf+5, buf+6, Speed);
    }

    SCSerial *scserial_;
};

}

#endif // HLSCL_H
