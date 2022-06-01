// See https://aka.ms/new-console-template for more information
using PID;


PIDController controller = new PIDController(1, 0, 0, 0, 1, 1);
float updated = controller.Update(2, 3, 5);

Console.WriteLine(updated);
