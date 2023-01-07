## Important Beginner Information
### Interfaces vs Abstract Classes
When WPILib was rewritten, the option of using an interface as opposed to an abstract class was added. This enables some
level of flexibility when writing Commands and Subsystems. The main difference is that an interface can only define types,
whereas an abstract class can define types **and** implement code.
#### `Subsystem` interface vs `SubsystemBase` abstract class
Subsystems have a `periodic` method that is called every 20ms. This method can be used to update state or whatever else
you may need.
In order for this periodic method to be called, the Subsystem must be registered with the `CommandScheduler`. When you
implement a subsystem with the `Subsystem` interface, you must register it yourself. When you extend the `SubsystemBase`
abstract class, the `SubsystemBase` constructor will register the subsystem for you.
#### `Command` interface vs `CommandBase` abstract class
Commands must require Subsystems in order to run. In order to function properly, the `CommandScheduler` must be able to
read what Subsystems a Command requires. When you implement a Command with the `Command` interface, you must use a `Set`
to store required subsystems. You must then have a method to return the `Set` of required subsystems. When you extend the
`CommandBase` abstract class, there is a built-in method called `addRequirements` that you call in the constructor to
specify required subsystems. This method abstract class then implements the rest behind the scenes.