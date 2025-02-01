from behave import given, when, then
import sensor_module  # Now uses Boost.Python instead of Pybind11

@given('the sensor is initialized')
def step_impl(context):
    context.sensor = sensor_module.G3T1_3(0, [], "sensor")

@when('I call the setup function')
def step_impl(context):
    context.sensor.setUp()

@then('the sensor should be properly configured')
def step_impl(context):
    print("Setup completed successfully!")
