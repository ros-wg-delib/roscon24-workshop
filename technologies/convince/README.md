# AS2FM by CONVINCE

This is an introduction to the tool [AS2FM (Autonomous Systems to Formal Models)](https://convince-project.github.io/AS2FM/index.html) that is part of the [CONVINCE toolchain](https://convince-project.github.io/overview/).
While there is also a [full documentation](https://convince-project.github.io/AS2FM/index.html), this is a short introductory example.

## Task 1: A battery that is drained but has an alarm

### Introduction 1

This is a simple example consisting of a battery that is drained at a fixed rate and the logic to charge it when it is low.

### System description 1

The folder `technologies/convince/task_1` contains all the necessary files for this example. In the folder you will find the following files:

- [`main.xml`](task_1/main.xml) - The main file referencing the other files and defining some global system parameters.
- __[`battery_drainer.scxml`](task_1/battery_drainer.scxml)__ - The state machine that models the battery which is drained.
- __[`battery_manager.scxml`](task_1/battery_manager.scxml)__ - Model of the manager that will evaluate the battery level.
- __[`battery_properties.jani`](task_1/battery_properties.jani)__ - The file defining the properties defined in [Linear Temporal Logic (LTL)](https://en.wikipedia.org/wiki/Linear_temporal_logic) to be checked by the model checker:
  - `battery_depleted`- The battery level is eventually <= 0.
  - `battery_over_depleted` - The battery level is eventually <= -1.
  - `alarm_on` - The alarm is eventually on.

### Task 1.1: Running the example

To run the example, navigate to the `task_1` folder:

```bash
cd $ROS_WS/src/technologies/convince/task_1
```

First, you need to translate all of the files mentioned above into a single JANI model file:

```bash
as2fm_scxml_to_jani main.xml
```

Note that there is now additionally a file `main.jani` in the folder.

Then, you can run the model checker on the generated JANI file, specifying the property to check:

```bash
smc_storm --model main.jani --properties-names battery_depleted
```

This will check the property `battery_depleted` and output the result.
We expect the property to hold, if our system is correct.
This means that in all the traces that the model checker explores, the battery level will eventually reach 0.
The output should look like this:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "battery_depleted": Pmin=? [F ((topic_level_msg.ros_fields__data <= 0) & topic_level_msg.valid)];
Result: 1
```

This means that the property holds with a probability of `1`, which is exactly what we expected.

Let's look at the property we get as an output in some more detail:

```txt
Pmin=? [F ((topic_level_msg.ros_fields__data <= 0) & topic_level_msg.valid)]
```

Here

- `Pmin=?` means that we are looking for the minimum probability that the property holds.
- `F` is the temporal operator _"finally"_ in [Linear Temporal Logic (LTL)](https://en.wikipedia.org/wiki/Linear_temporal_logic) (some people also call it _"eventually"_), which says the property holds at some point in the future.
- `((topic_level_msg.ros_fields__data <= 0) & topic_level_msg.valid)` is the logical expression that we are looking for:
  - `topic_level_msg.ros_fields__data <= 0` means that the battery level is less than or equal to 0. Note that this refers to the ROS topic `level` and its field `data`.
  - `topic_level_msg.valid` is something we add to the model to make sure the data has been sent at least once, otherwise we may check against uninitialized data.

### Task 1.2: Checking other properties

Without changing the model, you can also check the other properties by specifying them in the `smc_storm` command:

```bash
smc_storm --model main.jani --properties-names battery_over_depleted
```

This will check whether the battery level will eventually reach -1 or lower.
The output should look like this:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "battery_over_depleted": Pmin=? [F ((topic_level_msg.ros_fields__data <= -1) & topic_level_msg.valid)];
Result: 1
```

This is not what we expected, as the battery level should never reach -1.
Something is wrong with the model of the battery drainer.

You can also check the third property:

```bash
smc_storm --model main.jani --properties-names alarm_on
```

This should output:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "alarm_on": Pmin=? [(!(topic_level_msg.valid) | (topic_level_msg.ros_fields__data >= 29)) U (topic_alarm_msg.ros_fields__data & topic_alarm_msg.valid)];
Result: 0
```

This means that the property does not hold, and therefore our alarm does not work as expected.

### Task 1.3: Make sure that the battery can't drain below 0

To fix the model, you need to change the `battery_drainer.scxml` file.

In lines 22 to 24, you can see:

```xml
<ros_rate_callback name="my_timer" target="use_battery">
    <assign location="battery_percent" expr="battery_percent - 1" />
</ros_rate_callback>
```

This is a transition that gets triggered whenever the timer `my_timer` fires.
Because its target is `use_battery`, the state will not change.
But as you can see, the battery level is decreased by 1 every time the timer fires.
We need to make sure that the transition is not taken when the battery level is already 0.

- To do this, we can add a condition to the transition:
Add the attribute `cond="battery_percent > 0"` to the `<ros_rate_callback>` tag.
- Now run `as2fm_scxml_to_jani main.xml` and `smc_storm --model main.jani --properties-names battery_over_depleted` again.

The output should now be:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "battery_over_depleted": Pmin=? [F ((topic_level_msg.ros_fields__data <= -1) & topic_level_msg.valid)];
Result: 0
```

This means that the property does not hold anymore, which is what we expected: The battery level should never reach -1 or lower.

But this has not fixed the alarm yet. Feel free to running the model checker with the `alarm_on` property again. It should still output 0.

### Task 1.4: Enable the alarm

To enable the alarm, we need to make sure that the `battery_manager` is evaluating the battery level correctly.
Please convince yourself that the battery level is correctly sent.
This is done in lines 18 to 20 of the `battery_drainer.scxml` file:

```xml
<ros_topic_publish name="battery_level">
    <field name="data" expr="battery_percent" />
</ros_topic_publish>
```

It is received in the `battery_manager` in lines 22 to 24 of the `battery_manager.scxml` file:

```xml
<ros_topic_callback name="level" target="check_battery">
    <assign location="battery_alarm" expr="false" />
</ros_topic_callback>
```

This is again a transition that is triggered every time the topic `level` is received.
And it also does not change the state, because the target is `check_battery`.
But it assigns the value of `false` to the variable `battery_alarm`.
This is not what we want.
We want to set the alarm to `true` when the battery level is below 30.
So please change the `expr` attribute to `_msg.data &lt; 30`.

- Note that we need to use `&lt;` instead of `<` because of XML syntax.
- Note also that `_msg` is a variable that is automatically created to hold the content of the received ROS message.
- And the field `data` is the field of the `std_msgs/Int32` message that we are sending.

Now run `as2fm_scxml_to_jani main.xml` and `smc_storm --model main.jani --properties-names alarm_on` again.

The output should now be similar to this:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "alarm_on": Pmin=? [(!(topic_level_msg.valid) | (topic_level_msg.ros_fields__data >= 29)) U (topic_alarm_msg.ros_fields__data & topic_alarm_msg.valid)];
Result: 0.8164912281
```

(The exact number may vary slightly.)

We can see that the property holds with a probability of `0.8164912281`.

Let's look at the property in more detail:

```txt
Pmin=? [(!(topic_level_msg.valid) | (topic_level_msg.ros_fields__data >= 29)) U (topic_alarm_msg.ros_fields__data & topic_alarm_msg.valid)]
```

We now have a new [LTL](https://en.wikipedia.org/wiki/Linear_temporal_logic) operator `U` which means "until".
This means that, the left side of the expression must be true in all states before the right side becomes true.

- The left side is `(!(topic_level_msg.valid) | (topic_level_msg.ros_fields__data >= 29))` Checking again, that the data has been sent at least once and the battery level is above or equal to 29.
- The right side is `(topic_alarm_msg.ros_fields__data & topic_alarm_msg.valid)` Checking that the alarm is on.
- In some more plain English: The battery is above or equal 29 until the alarm is on.
- Or in other words: in 81.6% of the traces that the model checker explores, the battery level will be above 29 until the alarm is on.

### Task 1.5: Fix the alarm to be more reliable

Apparently, there are some traces where the battery is already below or equal to 29 when the alarm is turned on.
But there is a simple fix for this: We can change the condition in the `battery_manager` to `<=` (i.e. `&lt;=`) instead of `<`.

Your line 23 in the `battery_manager.scxml` file should now look like this:

```xml
<assign location="battery_alarm" expr="_msg.data &lt;= 30" />
```

Now run `as2fm_scxml_to_jani main.xml` and `smc_storm --model main.jani --properties-names alarm_on` again.
And you should see that the property holds with a probability of `1`:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "alarm_on": Pmin=? [(!(topic_level_msg.valid) | (topic_level_msg.ros_fields__data >= 29)) U (topic_alarm_msg.ros_fields__data & topic_alarm_msg.valid)];
Result: 1
```

This means that the alarm is now always on when the battery level is below or equal to 29.

### Conclusion 1

You have now successfully modeled a simple system with AS2FM and checked some properties with the model checker.
You have also learned that the modeling of distributed systems can lead to unexpected behavior and how to fix it.

## Task 2: Charging the battery using a behavior tree

### Introduction 2

In this task we keep the battery that is drained and the manager that evaluates the battery level.
But we add a feature to the drainer that can charge the battery.
The charging is controlled by a behavior tree.

### System description 2

- `main.xml` - The main file again referencing the other files and defining some global parameters.
- __`battery_drainer.scxml`__ - The state machine that models the battery which is drained _and can be charged_.
- __`battery_manager.scxml`__ - Model of the manager that will evaluate the battery level.
- __`bt.xml`__ - The behavior tree that implements the charging logic.
- __`bt_topic_action.scxml`__ - The model of the action plugin that allows the behavior tree to trigger ROS actions.
- __`bt_topic_condition.scxml`__ - The model of the condition plugin that allows the behavior tree to check ROS topics.
- __`battery_properties.jani`__ - The file defining the properties to be checked by the model checker:
  - `battery_depleted` - The battery is eventually depleted.
  - `battery_below_20` - The battery is eventually below 20%.
  - `battery_alarm_on` - The alarm is eventually on.
  - `battery_charged` - The battery will always eventually be at 100% again.

You will notice that there are some additional scxml files.
These implement the plugins that are used in the behavior tree.
They must communicate with the battery drainer and manager.

### Task 2.1: Running the example

Let's first run the existing example and see if the properties evaluate as expected.

```bash
cd $ROS_WS/src/technologies/convince/task_2
as2fm_scxml_to_jani main.xml
smc_storm --model main.jani --properties-names battery_depleted
```

This should output:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "battery_depleted": Pmin=? [F ((topic_level_msg.ros_fields__data <= 0) & topic_level_msg.valid)];
Result: 1
```

This means that the battery will eventually be depleted.
So we should enable the charging.

### Task 2.2: Enabling the charging

Currently, the battery is not charging, because the condition node that evaluates the alarm is not evaluated correctly.
If you inspect the `bt_topic_condition.scxml` file, you will see in line 23 that the condition is always false (i.e. we always return `FAILURE`).
But there is a variable in the state machine, called `last_msg`, that holds the last received message.
So we can replace line 23 by this:

```xml
<if cond="last_msg">
    <bt_return_status status="SUCCESS" />
    <else />
    <bt_return_status status="FAILURE" />
</if>
```

This will send the `SUCCESS` status if the last message was `true`, and the `FAILURE` event otherwise.

Now run `as2fm_scxml_to_jani main.xml` and `smc_storm --model main.jani --properties-names battery_charged` again.
The output should now be:

```txt
CONVINCE Statistical Model Checker
Checking model: main.jani
Property "battery_charged": Pmin=? [true Usteps>=100 ((topic_level_msg.ros_fields__data = 100) & topic_level_msg.valid)];
Result: 1
```

This means that the battery will always eventually be at 100% again.

### Conclusion 2

You have now successfully modeled a system that uses a behavior tree to control the charging of a battery.
This required you to implement a condition node plugin that evaluates the alarm correctly.
