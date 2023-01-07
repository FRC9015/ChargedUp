# `controllers` package
This package encapsulates classes that abstract the different controls of the robot. Each class may be customized for 
the given game. This allows for easy changes between different controller types (i.e. Joystick to Xbox Controller) as
well as easily swapping different control schemes and buttons, as all dependents see the same interface.

## Basic Structure
### Constructors
Each controller class has various constructors that allow for the use of different controllers. The constructor is
responsible for intaking an instance of a Joystick, XboxController, or other controller type. It then assigns the various
attributes of the controller to the correct buttons and axes.   
### Methods
Each controller class has a set of methods that allow for the use of the controller. These methods are named after the
function that they represent. For example, the `getShooterButton()` method returns the state of the button that controls
the shooter. These methods are also overloaded to return either a literal state of the button (e.g. `boolean`, `int`, 
`double`) or a `JoystickButton` instance that can be useful for binding commands.
## Notes
There are three distinct benefits to this structure as opposed to using built in WPILib stuff like `GenericHID`.
1. **Switching between different controller types.** At any point, you might want to change what controllers you're using.
Say your main Joysticks died, so you need to jump to an Xbox Controller. With this structure, you can simply change what
you pass into the constructor, and it will automatically assign the correct buttons and axes. This is much easier than 
checking your git history, commenting and uncommenting different code, or just straight up rewriting button bindings.
2. **Easy change between different controls depending on the controller used.** For example, you might want your
robot to drive arcade style with a joystick, but tank style with an Xbox controller. This is easily accomplished by
assigning attributes differently in the constructor.
3. **Bridging weird gaps between different controllers.** Dpads. Dpads on Xbox controllers are very different from Dpads
on PS4 controllers, and thus are represented differently in WPILib. This structure allows for easy abstraction of these differences.