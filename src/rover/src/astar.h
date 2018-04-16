


class AStar
{
public:
#pragma pack(1)
    struct State
    {
        short milliVolts;
        int left;
        int right;
    };
#pragma pack()

public:
    AStar();
    ~AStar();

    bool init();
    bool setMotor(int i2cFile, char left, char right);
    bool readState(int i2cFile, State &);

private:
    void readBuffer(int i2cFile, char reg, char * buff, int size);
};
