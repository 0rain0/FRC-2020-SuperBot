package frc.robot;

public class Utility {
  public static double Constrain(double Value,double max,double min){
    if(Value > max){
      Value = max;
    }else if(Value < min){
      Value = min;
    }
    return Value;
  }
  public static double Map(double Value,double InMax,double InMin,double OutMax,double OutMin){
    return (Value - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
  }
}
