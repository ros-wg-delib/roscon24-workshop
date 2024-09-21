# AS2FM by CONVINCE

This is an introduction to the tool AS2FM (Autonomous Systems to Formal Models) that is part of the CONVINCE toolchain.

The folder `example_w_bt` contains a model of a simplistic system that consist of a battery that is drained at a fixed rate and the logic to charge it when it is low.

In the folder you will find the following files:

- `main.xml` - The main file referencing the other files and defining some global system parameters.
- __`battery_drainer.scxml`__ - The state machine that models the battery which is drained.
- __`battery_manager.scxml`__ - Model of the manager that will evaluate the battery level.
- __`bt.xml`__ - The behavior tree that implements the charging logic.
- __`bt_topic_action.scxml`__ - The model of the action plugin that allows the behavior tree to trigger ROS actions.
- __`bt_topic_condition.scxml`__ - The model of the condition plugin that allows the behavior tree to check ROS topics.
- __`battery_properties.jani`__ - The file defining the properties to be checked by the model checker:
  - `battery_depleted` - The battery is eventually depleted.
  - `battery_below_20` - The battery is eventually below 20%.
  - `battery_alarm_on` - The alarm is eventually on.
  - `battery_charged` - The battery will always eventually be at 100% again.

To run the example, navigate to the `example_w_bt` folder:

```bash
cd /delib_ws/src/technologies/convince/example_w_bt
```

First, you need to translate all of the files mentioned above into a single JANI model file:

```bash
scxml_to_jani main.xml
```

Then, you can run the model checker on the generated JANI file, specifying the property to check:

```bash
smc_storm --model main.jani --properties-names battery_charged
```

This will check the property `battery_charged` and output the result of the model checking.