#pragma once

#include <rclcpp/rclcpp.hpp>

namespace rpi::parameters
{
using rcl_interfaces::msg::ParameterDescriptor;

// Primary template class with type restriction
template<typename T, typename Enable = void>
class AutoRosParameter;

// Specialization for supported types
template<typename T>
class AutoRosParameter<T, typename std::enable_if<
      std::is_arithmetic<T>::value || std::is_same<T, std::string>::value ||
      std::is_same<T, std::vector<bool>>::value || std::is_same<T, std::vector<int>>::value ||
      std::is_same<T, std::vector<double>>::value || std::is_same<T,
      std::vector<std::string>>::value>::type>
{
public:
  AutoRosParameter(rclcpp::Node & node)
  : node_(node) {}
  void createParameter(
    const std::string & param_name,
    const T & default_value,
    ParameterDescriptor descriptor)
  {
    param_name_ = param_name;
    // Declare the parameter
    node_.declare_parameter(param_name, default_value, descriptor);

    cb_handle_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters)
      {return callback(parameters);});

    // Get the initial value
    value_ = default_value;
  }

  T getValue() const
  {
    return value_;
  }

  // Operator overloads
  operator T() const
  {
    return getValue();
  }

  template<typename U = T, typename std::enable_if<std::is_arithmetic<U>::value, int>::type = 0>
  auto operator+(const U & other) const -> decltype(getValue() + other)
  {
    return getValue() + other;
  }

  template<typename U = T, typename std::enable_if<std::is_arithmetic<U>::value, int>::type = 0>
  auto operator-(const U & other) const -> decltype(getValue() - other)
  {
    return getValue() - other;
  }

  template<typename U = T, typename std::enable_if<std::is_arithmetic<U>::value, int>::type = 0>
  auto operator*(const U & other) const -> decltype(getValue() * other)
  {
    return getValue() * other;
  }

  template<typename U = T, typename std::enable_if<std::is_arithmetic<U>::value, int>::type = 0>
  auto operator/(const U & other) const -> decltype(getValue() / other)
  {
    return getValue() / other;
  }

  // Unary negation operator overload
  template<typename U = T, typename std::enable_if<std::is_arithmetic<U>::value, int>::type = 0>
  auto operator-() const -> U
  {
    return -getValue();
  }

private:
  rcl_interfaces::msg::SetParametersResult callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto & param : parameters) {
      if (param.get_name() == param_name_) {
        value_ = param.get_value<T>();
        result.successful = true;
        log_change(param);
      }
    }
    return result;
  }
  void log_change(const rclcpp::Parameter &) {}

  rclcpp::Node & node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
  std::string param_name_;
  T value_;
};

// Define callback specializations
template<>
inline void AutoRosParameter<bool>::log_change(const rclcpp::Parameter & param)
{
  RCLCPP_INFO(
    node_.get_logger(), "Bool parameter '%s' changed to '%s'",
    param.get_name().c_str(), param.as_bool() ? "true" : "false");
}

template<>
inline void AutoRosParameter<int>::log_change(const rclcpp::Parameter & param)
{
  RCLCPP_INFO(
    node_.get_logger(), "Int parameter '%s' changed to '%ld'",
    param.get_name().c_str(), param.as_int());
}

template<>
inline void AutoRosParameter<double>::log_change(const rclcpp::Parameter & param)
{
  RCLCPP_INFO(
    node_.get_logger(), "Double parameter '%s' changed to '%f'",
    param.get_name().c_str(), param.as_double());
}

template<>
inline void AutoRosParameter<std::string>::log_change(const rclcpp::Parameter & param)
{
  RCLCPP_INFO(
    node_.get_logger(), "String parameter '%s' changed to '%s'",
    param.get_name().c_str(), param.as_string().c_str());
}

template<>
inline void AutoRosParameter<std::vector<bool>>::log_change(const rclcpp::Parameter & param)
{
  std::string value;
  for (auto v : param.as_bool_array()) {
    value += (v ? "true" : "false") + std::string(" ");
  }
  RCLCPP_INFO(
    node_.get_logger(), "Bool array parameter '%s' changed to '%s'",
    param.get_name().c_str(), value.c_str());
}

template<>
inline void AutoRosParameter<std::vector<int>>::log_change(const rclcpp::Parameter & param)
{
  std::string value;
  for (auto v : param.as_integer_array()) {
    value += std::to_string(v) + ";";
  }
  RCLCPP_INFO(
    node_.get_logger(), "Int array parameter '%s' changed to '%s'",
    param.get_name().c_str(), value.c_str());
}

template<>
inline void AutoRosParameter<std::vector<double>>::log_change(const rclcpp::Parameter & param)
{
  std::string value;
  for (auto v : param.as_double_array()) {
    value += std::to_string(v) + ";";
  }
  RCLCPP_INFO(
    node_.get_logger(), "Double array parameter '%s' changed to '%s'",
    param.get_name().c_str(), value.c_str());
}

template<>
inline void AutoRosParameter<std::vector<std::string>>::log_change(const rclcpp::Parameter & param)
{
  std::string value;
  for (auto v : param.as_string_array()) {
    value += v + ";";
  }
  RCLCPP_INFO(
    node_.get_logger(), "String array parameter '%s' changed to '%s'",
    param.get_name().c_str(), value.c_str());
}

} /* namespace rpi::parameters */
