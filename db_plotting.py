from assembly import *
from PIL import Image
import os

db_sampler = AssemblyA_Sampler.load('/Users/shahafgoren/PycharmProjects/Mechanical_Characters/sampler_700')

def plot_all_db(self):
    for i, cur in enumerate(self.get_curve_database()):
        fig, ax = cur.plot_curve()
        fig.savefig(f"db_plots/{i}.png")

def get_asm(i):
    return db_sampler.get_curve_database()[i]

def plot_stick_figure(self, idx, figure='snake'):

    driving_mech = self.get_database()[idx]
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

    if not os.path.exists(f'temps/unnormalized_curve_{idx}'):
        print("generating unnormalized curve")
        curve = get_assembly_curve(combined)
        if not os.path.exists('temps/'):
            os.mkdir('temps/')
        with open(f'temps/unnormalized_curve_{idx}', 'wb') as f:
            pickle.dump(curve, f)
    else:
        with open(f'temps/unnormalized_curve_{idx}', 'rb') as f:
            curve = pickle.load(f)

    if not os.path.exists(f'images/{idx}'):
        os.mkdir(f'images/{idx}')

    fig_tup = curve.plot_curve()
    combined.plot_assembly(plot_path=f'images/{idx}/',
                           save_images=True,
                           image_number=0,
                           user_fig=figure,
                           fig_tup=fig_tup)

    print("generating motion images")

    for i in tqdm(range(72)):
        combined.actuator.turn(5)
        combined.update_state2()
        fig_tup = curve.plot_curve()
        combined.plot_assembly(plot_path=f'images/{idx}/',
                               save_images=True,
                               image_number=i + 1,
                               user_fig=figure,
                               fig_tup=fig_tup)
        plt.close()

    frames = []
    for i in range(72):
        new_frame = Image.open(f'images/{idx}/{i}.png')
        frames.append(new_frame)

    if not os.path.exists('gifs'):
        os.mkdir('gifs')
    # Save into a GIF file that loops forever
    frames[0].save(f'gifs/{idx}.gif', format='GIF',
                   append_images=frames[1:],
                   save_all=True,
                   duration=5, loop=0)


if __name__ == '__main__':
    plot_stick_figure(db_sampler, 201)
    plot_stick_figure(db_sampler, 235)
