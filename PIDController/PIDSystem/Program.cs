// See https://aka.ms/new-console-template for more information
using PID;

PIDController controller = new PIDController(5.0f, 0.04f, 0.05f, -1, 1, 1.0f);
float curr = 1000.76f;
float target = 100f;
float increment = 0.05f;
float gain;
while (true)
{
    gain = controller.Update(0.05f, curr, target);
    curr += increment * gain;

    Console.WriteLine(curr);
}