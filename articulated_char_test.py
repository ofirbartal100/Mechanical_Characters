from assembly import *
from matplotlib import pyplot as plt

figure = StickSnake()
figure.update_state2()
figure.plot_assembly()

driving_mech = return_prototype2()
driving_mech.update_state2()
driving_mech.plot_assembly()

combined = figure.add_driving_assembly(driving_mech)
combined.update_state2()
combined.plot_assembly(plot_path='/Users/shahafgoren/PycharmProjects/Mechanical_Characters/images/',
                       save_images=True,
                       image_number=0)

print(combined.describe_assembly())

for i in range(36):
    print(i)
    combined.actuator.turn(10)
    combined.update_state2()
    combined.plot_assembly(plot_path='/Users/shahafgoren/PycharmProjects/Mechanical_Characters/images/',
                           save_images=True,
                           image_number=i+1)
    print(combined.describe_assembly())
