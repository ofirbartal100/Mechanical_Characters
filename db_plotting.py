from assembly import *
from PIL import Image
import glob
import os

db_sampler = AssemblyA_Sampler.load('/Users/shahafgoren/PycharmProjects/Mechanical_Characters/sampler_700')

def make_plots():
    for i, cur in enumerate(db_sampler.get_curve_database()):
        fig, ax = cur.plot_curve()
        fig.savefig(f"db_plots/{i}.png")

def get_asm(i):
    return db_sampler.get_database()[i], db_sampler.get_curve_database()[i]

def plot_stick_figure(idx, figure='snake'):

    driving_mech, curve = get_asm(idx)
    driving_mech.update_state2()

    if figure is not None:
        if figure == 'man':
            figure = StickFigure()
        elif figure == 'snake':
            figure = StickSnake()
        figure.update_state2()
        combined = figure.add_driving_assembly(driving_mech)
        combined.update_state2()
    else:
        combined = driving_mech

    if not os.path.exists(f'images/{idx}'):
        os.mkdir(f'images/{idx}')
    combined.plot_assembly(plot_path=f'images/{idx}/',
                           save_images=True,
                           image_number=0,
                           user_fig=figure)

    print(combined.describe_assembly())

    for i in range(72):
        print(i)
        combined.actuator.turn(5)
        combined.update_state2()
        combined.plot_assembly(plot_path=f'images/{idx}/',
                               save_images=True,
                               image_number=i + 1,
                               user_fig=figure)
        print(combined.describe_assembly())

    frames = []
    for i in range(72):
        new_frame = Image.open(f'images/{idx}/{i}')
        frames.append(new_frame)

    # Save into a GIF file that loops forever
    frames[0].save('png_to_gif.gif', format='GIF',
                   append_images=frames[1:],
                   save_all=True,
                   duration=5, loop=0)

plot_stick_figure(151)
plot_stick_figure(8)
plot_stick_figure(600)
plot_stick_figure(475)
