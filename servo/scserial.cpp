#include "servo/scserial.h"

namespace feetech_servo
{

namespace {

struct ModelEntry
{
    QString name;
    uint8_t end;
};

struct FirmwareProfile
{
    uint8_t major;
    uint8_t minor_start;
    uint8_t minor_end;
    uint8_t end;
    feetech_servo::ModelSeries series;
};

// From setup.log [debug]. Firmware 3.20-3.39 appears twice, disambiguated by `end`.
//
// The 260623 ft_setup_bat also defines firmware families 5.x (CWSXX), 6.x
// (LYNODE), 7.x (CWSR) and 8.x (TTSD). They are deliberately absent here: no
// register tables have been ported for them, so they resolve as unknown and
// fail closed rather than being driven through a guessed map.
const std::vector<FirmwareProfile> firmware_profiles =
{
    { 0,  0, 39, 1, feetech_servo::SCS  },
    { 1,  0, 19, 0, feetech_servo::SMCL },
    // 1.20-1.39 is present in the 250729 config and dropped in 260623. Kept,
    // because retaining it can only let an older SMCL servo resolve, whereas
    // dropping it would regress one to fail-closed.
    { 1, 20, 39, 0, feetech_servo::SMCL },
    { 2, 40, 69, 0, feetech_servo::SMBL },
    { 3,  0, 39, 0, feetech_servo::STS  },
    { 3, 20, 39, 1, feetech_servo::SCS2 },
    { 3, 40, 59, 0, feetech_servo::HLS  },
};

#define SERVO_MODEL(major, minor, name, end) { (uint16_t)((minor)<<8 | (major)), ModelEntry{name, end} }
const std::map<uint16_t, ModelEntry> &model_list()
{
    static const std::map<uint16_t, ModelEntry> list =
    {
        SERVO_MODEL(1, 1, "TTL-Node-A", 0),
        SERVO_MODEL(5, 0, "SCSXX", 1),
        SERVO_MODEL(5, 1, "SCS0002", 1),
        SERVO_MODEL(5, 2, "SCS0037", 1),
        SERVO_MODEL(5, 3, "SCS2304", 1),
        SERVO_MODEL(5, 4, "SCS009", 1),
        SERVO_MODEL(5, 5, "SCS1025", 1),
        SERVO_MODEL(5, 6, "SCS0018", 1),
        SERVO_MODEL(5, 7, "SCS0017", 1),
        SERVO_MODEL(5, 8, "SCS2332", 1),
        SERVO_MODEL(5, 9, "SCS0005", 1),
        SERVO_MODEL(5, 10, "SCS0043", 1),
        SERVO_MODEL(5, 12, "SCS45", 1),
        SERVO_MODEL(5, 15, "SCS15", 1),
        SERVO_MODEL(5, 16, "SCS315", 1),
        SERVO_MODEL(5, 25, "SCS115", 1),
        SERVO_MODEL(5, 35, "SCS215", 1),
        SERVO_MODEL(5, 40, "SCS40", 1),
        SERVO_MODEL(5, 60, "SCS6560", 1),
        SERVO_MODEL(6, 0, "SMXX-360M", 0),
        SERVO_MODEL(6, 4, "SM30-360M", 0),
        SERVO_MODEL(6, 8, "SM60-360M", 0),
        SERVO_MODEL(6, 12, "SM80-360M", 0),
        SERVO_MODEL(6, 16, "SM100-360M", 0),
        SERVO_MODEL(6, 20, "SM150-360M", 0),
        SERVO_MODEL(6, 24, "SM85-360M", 0),
        SERVO_MODEL(6, 26, "SM60-360M", 0),
        SERVO_MODEL(8, 10, "SM30BL", 0),
        SERVO_MODEL(8, 16, "SM100-360M", 0),
        SERVO_MODEL(8, 20, "SM150-360M", 0),
        SERVO_MODEL(8, 24, "SM24BL", 0),
        SERVO_MODEL(8, 25, "SM70BLHV", 0),
        SERVO_MODEL(8, 29, "SM29BL", 0),
        SERVO_MODEL(8, 30, "SM30BL", 0),
        SERVO_MODEL(8, 40, "SM40BLHV", 0),
        SERVO_MODEL(8, 41, "SM80BLHV", 0),
        SERVO_MODEL(8, 42, "SM45BLHV", 0),
        SERVO_MODEL(8, 44, "SM85BLHV", 0),
        SERVO_MODEL(8, 81, "SM160BLHV", 0),
        SERVO_MODEL(8, 105, "SM105BLHV", 0),
        SERVO_MODEL(8, 120, "SM120BLHV", 0),
        SERVO_MODEL(8, 121, "SM260BLHV", 0),
        SERVO_MODEL(8, 220, "SM200BLHV", 0),
        SERVO_MODEL(8, 224, "SM224BLHV", 0),
        SERVO_MODEL(9, 0, "STSXX", 0),
        SERVO_MODEL(9, 1, "STS3015", 0),
        SERVO_MODEL(9, 2, "STS3032", 0),
        SERVO_MODEL(9, 3, "STS3215", 0),
        SERVO_MODEL(9, 4, "STS3040", 0),
        SERVO_MODEL(9, 5, "STS3020", 0),
        SERVO_MODEL(9, 6, "STS3046", 0),
        SERVO_MODEL(9, 7, "STS3045", 0),
        SERVO_MODEL(9, 8, "STS3235", 0),
        SERVO_MODEL(9, 9, "STS3095", 0),
        SERVO_MODEL(9, 10, "STS3095", 0),
        SERVO_MODEL(9, 11, "STS3250", 0),
        SERVO_MODEL(9, 12, "STS3036", 0),
        SERVO_MODEL(9, 13, "STS3120", 0),
        SERVO_MODEL(9, 15, "SCS15-2", 1),
        SERVO_MODEL(9, 20, "SCSXX-2", 1),
        SERVO_MODEL(9, 25, "SCS215-2", 1),
        SERVO_MODEL(9, 35, "SCS225", 1),
        SERVO_MODEL(9, 40, "SCS40-2", 1),
        SERVO_MODEL(9, 41, "SCS25-2", 1),
        SERVO_MODEL(9, 46, "SCS46-2", 1),
        SERVO_MODEL(10, 1, "HTS3235", 0),
        SERVO_MODEL(10, 2, "HTS3032", 0),
        SERVO_MODEL(10, 3, "HTS3240", 0),
        SERVO_MODEL(10, 4, "HTS3235", 0),
        SERVO_MODEL(10, 5, "STS3045BL", 0),
        SERVO_MODEL(10, 6, "STS3046BL", 0),
        SERVO_MODEL(10, 7, "HTS3032", 0),
        SERVO_MODEL(10, 8, "HTS3045", 0),
        SERVO_MODEL(10, 9, "STS3009BL", 0),
        SERVO_MODEL(10, 10, "HLS3606", 0),
        SERVO_MODEL(10, 11, "HLS3612", 0),
        SERVO_MODEL(10, 12, "HLS3620", 0),
        SERVO_MODEL(10, 13, "HLS3625", 0),
        SERVO_MODEL(10, 14, "HLS3640", 0),
        SERVO_MODEL(10, 15, "HLS3925", 0),
        SERVO_MODEL(10, 16, "HLS3930", 0),
        SERVO_MODEL(10, 17, "HLS3935", 0),
        SERVO_MODEL(10, 18, "HLS3950", 0),
        SERVO_MODEL(10, 19, "HLS3955", 0),
        SERVO_MODEL(10, 20, "HLS3915", 0),
        SERVO_MODEL(10, 21, "HLS3615", 0),
        SERVO_MODEL(10, 22, "HLS3960", 0),
        SERVO_MODEL(10, 23, "HLS3608", 0),
        SERVO_MODEL(10, 24, "HLS3604", 0),
        SERVO_MODEL(10, 25, "STS3025BL", 0),
        SERVO_MODEL(10, 26, "STS3200BL", 0),
        SERVO_MODEL(10, 27, "HLS2915", 0),
        SERVO_MODEL(10, 28, "HLS3906", 0),
        SERVO_MODEL(10, 29, "HLS2606", 0),
        SERVO_MODEL(11, 1, "SWS3225", 0),
        SERVO_MODEL(11, 101, "TTL_E02", 0),
        SERVO_MODEL(11, 102, "TTL_E02", 0),
        SERVO_MODEL(12, 1, "SR3307", 0),
        SERVO_MODEL(13, 1, "TTL_SD01", 0),
    };
    return list;
}
#undef SERVO_MODEL

}

ServoProfile resolveServo(uint16_t model_number, uint16_t firmware_version)
{
    ServoProfile p;
    p.name   = "Unknown";
    p.series = UNKNOWN;
    p.end    = 0;
    p.known  = false;

    const auto &models = model_list();
    auto it = models.find(model_number);
    if(it != models.end())
    {
        p.name = it->second.name;
        p.end  = it->second.end;
    }

    const uint8_t fw_major = firmware_version & 0xff;
    const uint8_t fw_minor = (firmware_version >> 8) & 0xff;

    for(const auto &fp : firmware_profiles)
    {
        if(fp.major != fw_major)
            continue;
        if(fw_minor < fp.minor_start || fp.minor_end < fw_minor)
            continue;
        // When the model is unknown its endianness is unknown too, so accept the
        // first firmware match. When the model IS known, the flag disambiguates
        // the overlapping 3.20-3.39 range.
        if(it != models.end() && fp.end != p.end)
            continue;

        p.series = fp.series;
        p.end    = fp.end;
        p.known  = true;
        if(it == models.end())
            p.name = QString("Unknown (fw %1.%2)").arg(fw_major).arg(fw_minor);
        break;
    }

    return p;
}

SCSerial::SCSerial(QSerialPort *serial)
    : serial_(serial)
{
    level_ = 1;
    error_ = 0;
    end_ = 0;
}

int SCSerial::gen_write(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len)
{
    read_flush();
    write_buf(id, mem_addr, n_dat, n_len, INST_WRITE);
    write_flush();
    return ask(id);
}

int SCSerial::reg_write(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len)
{
    read_flush();
    write_buf(id, mem_addr, n_dat, n_len, INST_REG_WRITE);
    write_flush();
    return ask(id);
}

int SCSerial::reg_write_action(uint8_t id)
{
    read_flush();
    write_buf(id, 0, NULL, 0, INST_REG_ACTION);
    write_flush();
    return ask(id);
}

void SCSerial::sync_write(uint8_t id[], uint8_t idn, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len)
{
    read_flush();
    uint8_t mes_len = ((n_len+1)*idn+4);
    uint8_t sum = 0;
    uint8_t b_buf[7];
    b_buf[0] = 0xff;
    b_buf[1] = 0xff;
    b_buf[2] = 0xfe;
    b_buf[3] = mes_len;
    b_buf[4] = INST_SYNC_WRITE;
    b_buf[5] = mem_addr;
    b_buf[6] = n_len;
    write(b_buf, 7);

    sum = 0xfe + mes_len + INST_SYNC_WRITE + mem_addr + n_len;
    uint8_t i, j;
    for(i=0; i<idn; i++){
        write(id[i]);
        write(n_dat+i*n_len, n_len);
        sum += id[i];
        for(j=0; j<n_len; j++){
            sum += n_dat[i*n_len+j];
        }
    }
    write(~sum);
    write_flush();
}

int SCSerial::write_byte(uint8_t id, uint8_t mem_addr, uint8_t b_dat)
{
    read_flush();
    write_buf(id, mem_addr, &b_dat, 1, INST_WRITE);
    write_flush();
    return ask(id);
}

int SCSerial::write_word(uint8_t id, uint8_t mem_addr, uint16_t w_dat)
{
    uint8_t b_buf[2];
    host_2_scs(b_buf+0, b_buf+1, w_dat);
    read_flush();
    write_buf(id, mem_addr, b_buf, 2, INST_WRITE);
    write_flush();
    return ask(id);
}

int SCSerial::read(uint8_t id, uint8_t mem_addr, uint8_t *n_data, uint8_t n_len)
{
    read_flush();
    write_buf(id, mem_addr, &n_len, 1, INST_READ);
    write_flush();
    if(!check_head()){
        return 0;
    }
    uint8_t b_buf[4];
    error_ = 0;
    if(read(b_buf, 3)!=3){
        return 0;
    }
    int size = read(n_data, n_len);
    if(size!=n_len){
        return 0;
    }
    if(read(b_buf+3, 1)!=1){
        return 0;
    }
    uint8_t cal_sum = b_buf[0]+b_buf[1]+b_buf[2];
    uint8_t i;
    for(i=0; i<size; i++){
        cal_sum += n_data[i];
    }
    cal_sum = ~cal_sum;
    if(cal_sum!=b_buf[3]){
        return 0;
    }
    error_ = b_buf[2];
    return size;
}

int SCSerial::read_byte(uint8_t id, uint8_t mem_addr)
{
    uint8_t b_dat;
    int size = read(id, mem_addr, &b_dat, 1);
    if(size!=1){
        return -1;
    }else{
        return b_dat;
    }
}

int SCSerial::read_word(uint8_t id, uint8_t mem_addr)
{
    uint8_t n_dat[2];
    int size;
    uint16_t w_dat;
    size = read(id, mem_addr, n_dat, 2);
    if(size!=2)
        return -1;
    w_dat = scs_2_host(n_dat[0], n_dat[1]);
    return w_dat;
}

int SCSerial::ping(uint8_t id)
{
    read_flush();
    write_buf(id, 0, NULL, 0, INST_PING);
    write_flush();
    error_ = 0;
    if(!check_head()){
        return -1;
    }
    uint8_t b_buf[4];
    if(read(b_buf, 4)!=4){
        return -1;
    }
    if(b_buf[0]!=id && id!=0xfe){
        return -1;
    }
    if(b_buf[1]!=2){
        return -1;
    }
    uint8_t cal_sum = ~(b_buf[0]+b_buf[1]+b_buf[2]);
    if(cal_sum!=b_buf[3]){
        return -1;			
    }
    error_ = b_buf[2];
    return b_buf[0];
}

void SCSerial::write_buf(uint8_t id, uint8_t mem_addr, uint8_t *n_dat, uint8_t n_len, uint8_t fun)
{
    uint8_t msg_len = 2;
    uint8_t b_buf[6];
    uint8_t check_sum = 0;
    b_buf[0] = 0xff;
    b_buf[1] = 0xff;
    b_buf[2] = id;
    b_buf[4] = fun;
    if(n_dat){
        msg_len += n_len + 1;
        b_buf[3] = msg_len;
        b_buf[5] = mem_addr;
        write(b_buf, 6);
        
    }else{
        b_buf[3] = msg_len;
        write(b_buf, 5);
    }
    check_sum = id + msg_len + fun + mem_addr;
    uint8_t i = 0;
    if(n_dat){
        for(i=0; i<n_len; i++){
            check_sum += n_dat[i];
        }
        write(n_dat, n_len);
    }
    write(~check_sum);
}

void SCSerial::host_2_scs(uint8_t *data_l, uint8_t* data_h, uint16_t data)
{
    if(end_){
        *data_l = (data>>8);
        *data_h = (data&0xff);
    }else{
        *data_h = (data>>8);
        *data_l = (data&0xff);
    }
}

uint16_t SCSerial::scs_2_host(uint8_t data_l, uint8_t data_h)
{
    uint16_t data;
    if(end_){
        data = data_l;
        data<<=8;
        data |= data_h;
    }else{
        data = data_h;
        data<<=8;
        data |= data_l;
    }
    return data;
}

int	SCSerial::ask(uint8_t id)
{
    error_ = 0;
    if(id!=0xfe && level_){
        if(!check_head()){
            return 0;
        }
        uint8_t b_buf[4];
        if(read(b_buf, 4)!=4){
            return 0;
        }
        if(b_buf[0]!=id){
            return 0;
        }
        if(b_buf[1]!=2){
            return 0;
        }
        uint8_t cal_sum = ~(b_buf[0]+b_buf[1]+b_buf[2]);
        if(cal_sum!=b_buf[3]){
            return 0;			
        }
        error_ = b_buf[2];
    }
    return 1;
}

int SCSerial::check_head()
{
    uint8_t b_dat;
    uint8_t b_buf[2] = {0, 0};
    uint8_t cnt = 0;
    while(1){
        if(!read(&b_dat, 1)){
            return 0;
        }
        b_buf[1] = b_buf[0];
        b_buf[0] = b_dat;
        if(b_buf[0]==0xff && b_buf[1]==0xff){
            break;
        }
        cnt++;
        if(cnt>10){
            return 0;
        }
    }
    return 1;
}

int SCSerial::read_model_number(int id)
{
    int tmp = -1;
    int model_number = -1;
    {
        error_ = 0;
        tmp = read_byte(id, 3); // 3 : Servo Main Version, 4 : Servo Sub Version
        if(tmp==-1){
            error_ = 1;
        }
        else
        {
            model_number = tmp;
            {
                tmp = read_byte(id, 4);
                if(tmp==-1){
                    error_ = 1;
                }
                else
                {
                    model_number |= tmp<<8;
                }
            }
        }
    }
    return model_number;
}

// Addresses 0 and 1 hold the firmware version, which is what selects the
// register map (the model number at 3/4 only names the servo).
int SCSerial::read_firmware_version(int id)
{
    error_ = 0;

    int major = read_byte(id, 0);
    if(major == -1)
    {
        error_ = 1;
        return -1;
    }

    int minor = read_byte(id, 1);
    if(minor == -1)
    {
        error_ = 1;
        return -1;
    }

    return (minor << 8) | major;
}

int SCSerial::write(uint8_t *n_dat, int n_len) {
    return serial_->write(reinterpret_cast<const char *>(n_dat), n_len);
}

int SCSerial::read(uint8_t *n_dat, int n_len) {
    if(serial_->bytesAvailable() < n_len)
        serial_->waitForReadyRead(100);
    return serial_->read(reinterpret_cast<char *>(n_dat), n_len);
}

int SCSerial::write(uint8_t b_dat) {
    return serial_->write(reinterpret_cast<const char *>(&b_dat), 1);
}

} // namespace feetech_servo
