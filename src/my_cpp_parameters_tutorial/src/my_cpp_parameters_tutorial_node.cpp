#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MyMinimalParam : public rclcpp::Node
{
public:
  // Question (AT): So we are setting up the constructor to create a default node = minimal param node
  // then setting up a param
  // and creating a timer that calls then callback every 1000ms ?

  // Answer (AT): Was I correct about the above? 
  // Yes my thought was correct, that said there is extra detail. The parameter
  // type is set to "world" as a default, and the type is inferred from this default.
  // This is just extra semantics but I think it is important to be clear about this.
  MyMinimalParam()
    : Node("my_minimal_param_node")
    {
      // create a definition for the param programatically
      auto param_description = rcl_interfaces::msg::ParameterDescriptor{};
      param_description.description = "Example defining a param description!";

      // Is this declaring then setting the value?
      this->declare_parameter("my_parameter", "world", param_description);
 
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&MyMinimalParam::timer_callback, this)
      );
    }

  // Question (AT): So the callback seems to get the param, log then param,
  // then create all new params, BUT it looks like it is first setting a single string
  // then setting it to an array of values?
  // Answer (AT): Was I correct about the above? 

  // NOTE (AT): personally I prefer this function definition syntax better than
  // defining the void first. 
  // Interestingly not what I would expect from reading.
  // This code is supposed to get the value, log it, then
  // ensure the value is correctly set to "world" again in the
  // event someone changed it externally. 
  // The way it is written is confusing. It might be better to write is as
  /*

    // Where this is getting the expected value, instead of making a 
    // default value hard coded in two places.
    // Mainly because if the default ever needed to change, the dev
    // would need to know to check here as well for where the code "Could set values back to default"
    // If they forgot then you would introduce a bug.
    std::vector<rclcpp::Parameter> default_params{
        rclcpp::Parameter("my_parameter", my_param.c_str())};
    this->set_parameters(default_params)

   */
  auto timer_callback() -> void
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    // I need to read more about this logger and how the default nodes use them.
    // Seems really nice to have all of this rolled for me, but does it actually do
    // everything I need a logger to do?
    // - live write data to a file of choice?
    // - thread safe?
    // - etc?
    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    // NOTE: So this would basically allow me to change a param once. Then it goes back to default
    // Good for on demand changes?
    // std::vector<rclcpp::Parameter> all_new_parameters{
    //     rclcpp::Parameter("my_parameter", "world")};
    // this->set_parameters(all_new_parameters);

    // NOTE: Whereas this allows me to change a param once and it Becomes the default.
    // Good for launch file params
    std::vector<rclcpp::Parameter> all_new_parameters{
        rclcpp::Parameter("my_parameter", my_param)};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

// Question (AT): Is spin just spinning this up as a service?
// or is it "spinning" the main thread until MyMinimalParam is killed?
auto main (int argc, char ** argv) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyMinimalParam>());
  rclcpp::shutdown();
  return 0;
}