from assembly import *
from PIL import Image
import os
import dill
from os.path import join as pjoin


class AssemblyA_Sampler:
    def __init__(self, number_of_points=72, num_of_samples_around=10):
        self.database = []
        self.curve_database = []
        self.number_of_points = number_of_points
        self.num_of_samples_around = num_of_samples_around

    def recursive_sample_assemblyA(self, assemblyA, num_of_samples_around=None, debug_mode=False, second_type=False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        accepted_assemblies = []
        for i in range(num_of_samples_around):
            new_assemblyA = sample_from_cur_assemblyA(assemblyA, random_sample=0.5, second_type=second_type)
            if is_vaild_assembleA(new_assemblyA, debug_mode=debug_mode):
                if debug_mode:
                    print("valid assembly!")
                assembly_curve = get_assembly_curve(new_assemblyA, number_of_points=self.number_of_points,
                                                    normelaize_curve=True)
                if is_dissimilar(assembly_curve, self.curve_database):
                    if debug_mode:
                        print(f"----------------added assembly {len(self.curve_database)}----------------")
                    self.curve_database.append(assembly_curve)
                    accepted_assemblies.append(new_assemblyA)
                elif (debug_mode):
                    print(f"assembly too similar to db")

        return accepted_assemblies

    def get_origin_assembly(self, second_type=False):
        origin_assembly = create_assemblyA(second_type=second_type)
        origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                          normelaize_curve=True)

        while not is_dissimilar(origin_curve, self.curve_database):
            origin_assembly = create_assemblyA(second_type=second_type)
            origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                              normelaize_curve=True)

        self.database += [origin_assembly]
        self.curve_database += [origin_curve]

        return origin_assembly, origin_curve

    def create_assemblyA_database(self, min_samples_number=1000, num_of_samples_around=None, debug_mode=False,
                                  second_type=False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        cur_database_len = len(self.database)

        origin_assembly, _ = self.get_origin_assembly(second_type=second_type)

        if debug_mode:
            print(f"origin_assembly initiaized")
            print(f"curve_database is {self.curve_database}")

        while len(self.database) - cur_database_len < min_samples_number:
            if debug_mode:
                print(f"current database size {len(self.database)}")
            accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                  num_of_samples_around=num_of_samples_around,
                                                                  debug_mode=debug_mode, second_type=second_type)
            self.database += accepted_assemblies
            while len(accepted_assemblies) > 0 and len(self.database) < min_samples_number:
                origin_assembly = accepted_assemblies[0]
                neighbor_accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                               num_of_samples_around=num_of_samples_around,
                                                                               debug_mode=debug_mode,
                                                                               second_type=second_type)
                self.database += neighbor_accepted_assemblies
                accepted_assemblies = accepted_assemblies[1:]
                accepted_assemblies += neighbor_accepted_assemblies
            if debug_mode:
                print("---we will get another origin assembly---")
            origin_assembly, _ = self.get_origin_assembly()

    def get_database(self):
        return self.database

    def get_curve_database(self):
        return self.curve_database

    def get_closest_curve(self, curve, get_all_dis=False):

        min_dis = curve.normA(curve, self.curve_database[0])
        min_curve = self.curve_database[0]
        closest_assembly = self.database[0]
        if get_all_dis:
            all_dist = {}
        for i, db_curve in enumerate(self.curve_database[1:]):
            cur_dis = curve.normA(curve, db_curve)
            if get_all_dis:
                all_dist[db_curve] = cur_dis
            if cur_dis < min_dis:
                closest_assembly = self.database[i + 1]
                min_dis = cur_dis
                min_curve = db_curve

        return min_curve, closest_assembly, all_dist if get_all_dis else None

    def save(self, path=pjoin('dbs', 'new_db')):
        with open(path, "wb") as handle:
            dill.dump(self, handle)

    @staticmethod
    def load(destination_path):
        with open(destination_path, 'rb') as input_file:
            sample = dill.load(input_file)
        return sample

    def plot_all_db(self):
        if not os.path.exists('db_plots'):
            #     os.rmdir('db_plots')
            os.mkdir('db_plots')
        for i, cur in enumerate(self.get_curve_database()):
            fig, ax = cur.plot_curve()
            fig.savefig(pjoin('db_plots', f'{i}.png'))

    def plot_with_figure(self, idx, figure='snake'):

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

        if not os.path.exists(pjoin('unnormalized_curves', f'{idx}')):
            print("generating unnormalized curve")
            curve = get_assembly_curve(combined)
            if not os.path.exists('unnormalized_curves'):
                os.mkdir('unnormalized_curves')
            with open(pjoin('unnormalized_curves', f'{idx}'), 'wb') as f:
                dill.dump(curve, f)
        else:
            with open(pjoin('unnormalized_curves', f'{idx}'), 'rb') as f:
                curve = dill.load(f)

        if not os.path.exists('images'):
            os.mkdir('images')
        if not os.path.exists(pjoin('images', f'{idx}')):
            os.mkdir(pjoin('images', f'{idx}'))

        fig_tup = curve.plot_curve()
        combined.plot_assembly(plot_path=pjoin('images', f'{idx}'),
                               save_images=True,
                               image_number=0,
                               user_fig=figure,
                               fig_tup=fig_tup)

        print("generating motion images")

        for i in tqdm(range(72)):
            combined.actuator.turn(5)
            combined.update_state2()
            fig_tup = curve.plot_curve()
            combined.plot_assembly(plot_path=pjoin('images', f'{idx}'),
                                   save_images=True,
                                   image_number=i + 1,
                                   user_fig=figure,
                                   fig_tup=fig_tup)
            plt.close()

        frames = []
        for i in range(72):
            new_frame = Image.open(pjoin('images', f'{idx}', f'{i}.png'))
            frames.append(new_frame)

        if not os.path.exists('gifs'):
            os.mkdir('gifs')
        # Save into a GIF file that loops forever
        frames[0].save(pjoin('gifs', f'{idx}.gif'), format='GIF',
                       append_images=frames[1:],
                       save_all=True,
                       duration=5, loop=0)
