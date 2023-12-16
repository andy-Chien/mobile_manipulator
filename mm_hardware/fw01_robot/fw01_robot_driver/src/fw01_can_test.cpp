#include "fw01_robot_driver/fw01_can.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"



int main (int argc, char* argv[]) {
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "Program Start");
 
  rclcpp::init(argc, argv);

  fw01_robot_driver::YhsCanControl fw01_can_("can0", std::make_shared<rclcpp::Logger>(rclcpp::get_logger("fw01_can_test"))); 
  //fw01_can_.openConnection("can0"); // auto-open during construction
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "Open Suceess");

  for (int j=0; j<1000000; j++) 
    fw01_can_.processUpdates();

  bool io_data[43];
  for (int j=0; j<43 ;j++) {
    fw01_can_.readIO(j, io_data[j]);
  }
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "IO_Data[0, 1, 8, 10, 11, 12] : [%d,  %d, %d, %d, %d, %d]", io_data[0],io_data[1],io_data[8],io_data[10],io_data[11],io_data[12]);
  
  fw01_can_.setIO(0,true);
  fw01_can_.setIO(8,false);
  fw01_can_.setIO(10,false);
  fw01_can_.setIO(11,false);
  fw01_can_.setIO(12,false);
  fw01_can_.writeIOs();
  //fw01_can_.writeIOs();
  //  fw01_can_.writeIOs();
  //    fw01_can_.writeIOs();  

  fw01_can_.processUpdates();
  for (int j=0; j<43 ;j++) {
    fw01_can_.readIO(j, io_data[j]);
  }
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "IO_Data[0, 1, 8, 10, 11, 12] : [%d, %d, %d, %d, %d, %d]", io_data[0],io_data[1],io_data[8],io_data[10],io_data[11],io_data[12]);

  uint16_t volt, cap;
  int16_t cur;
  fw01_can_.readBMS(volt, cur, cap);
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "BMS_FB Volt: %lf, Current: %lf, Remaining Capacity: %lf", volt/100.0, cur/100.0, cap/100.0);



  for (int j=0; j<1000000; j++) 
    fw01_can_.processUpdates();

  for (int j=0; j<43 ;j++) {
    fw01_can_.readIO(j, io_data[j]);
  }
  RCLCPP_INFO(rclcpp::get_logger("fw01_can_test"), "IO_Data[0, 1, 8, 10, 11, 12] : [%d, %d, %d, %d, %d, %d]", io_data[0],io_data[1],io_data[8],io_data[10],io_data[11],io_data[12]);


  fw01_can_.closeConnection();
  rclcpp::shutdown();
  return 0;
}