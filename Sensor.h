
class Sensor{
  private:
    int ID;
    float value;
    
  public:
    bool Connect(int ID);
    float Read();
    float GetLatestVal();
    float GetID();
};
