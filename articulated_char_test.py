from assembly import *
from matplotlib import pyplot as plt

figure = StickFigure()
figure.update_state2()
figure.plot_assembly()
driving_mech = return_prototype()
driving_mech.update_state2()
driving_mech.plot_assembly()

combined = figure.add_driving_assembly(driving_mech)
combined.update_state2()
combined.plot_assembly(plot_path='/Users/shahafgoren/PycharmProjects/Mechanical_Characters/images/',
                       save_images=True,
                       image_number=0,
                       user_fig=figure)

print(combined.describe_assembly())

for i in range(72):
    print(i)
    combined.actuator.turn(5)
    combined.update_state2()
    combined.plot_assembly(plot_path='/Users/shahafgoren/PycharmProjects/Mechanical_Characters/images/',
                           save_images=True,
                           image_number=i + 1,
                           user_fig=figure)
    print(combined.describe_assembly())
