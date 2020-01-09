from connections import *
from parts import *
from configuration import *
from assembly import Assembly
import time

actuator = Actuator()
gear1 = Gear(radius=10)
gear2 = Gear(radius=1)
stick1 = Stick(length=20)
assembly = Assembly([PhaseConnection(actuator, gear1),
                     FixedConnection(gear1, Point(0, 0, 0), Alignment(0, 0, None)),
                     PinConnection(gear1, stick1, Point(5, 0, 0), Point(0, 0, 0)),
                     ])

for i in range(360):
    actuator.turn(1)
    print("success: ", assembly.update_state())
    t = time.time()
    print('actuator turned: ', i)
    print('gear1 orientation:', gear1.configuration.alignment.vector())
    print('gear1 position:', gear1.configuration.position.vector())
    print('gear2 orientation:', gear2.configuration.alignment.vector())
    print('gear2 position:', gear2.configuration.position.vector())
    print('stick1 orientation:', stick1.configuration.alignment.vector())
    print('stick1 position:', stick1.configuration.position.vector())
    print("***************************************************************")
    print("time: ", time.time()-t)



# balanced 0.36
# time:  0.0006458759307861328
#       fun: 0.25145174556366306
#  hess_inv: array([[ 6.08462930e-01,  4.03370531e-04,  7.33491485e-02,
#          2.69293145e-03, -1.24812932e-03,  5.51578172e-05,
#         -3.27410841e-03,  4.42706601e-01, -6.01828966e-03,
#          4.40092680e-01,  3.47452316e-03, -6.33217856e-02,
#          3.92757139e-03, -2.25249785e-01, -5.20765089e-04,
#          3.86398138e-04],
#        [ 4.03370531e-04,  4.11725218e-01, -2.99602576e-03,
#         -2.07009881e-04,  1.09687043e-04, -2.68761727e-06,
#          3.02049698e-01, -4.31312854e-03,  8.95864332e-04,
#         -2.47747467e-02,  1.60608358e-01,  7.60594167e-04,
#         -5.09218324e-04,  2.03207051e-03,  1.95015914e-04,
#          2.94764794e-04],
#        [ 7.33491485e-02, -2.99602576e-03,  6.50837423e-01,
#          1.40179977e-02, -6.35384035e-03,  2.16870228e-04,
#         -4.48098547e-03,  8.15803062e-01, -2.45300129e-02,
#          9.88780298e-01,  1.25919942e-02, -1.20367839e-01,
#          1.53769205e-02, -4.16909979e-01, -1.42411656e-03,
#          9.29943710e-04],
#        [ 2.69293145e-03, -2.07009881e-04,  1.40179977e-02,
#          4.88427650e-01,  2.35206584e-01,  1.27926217e-04,
#          3.08292058e-04,  1.90503255e-02,  3.02909937e-01,
#          8.00069322e-01,  2.53432233e-03,  3.45196275e-02,
#          2.37643311e-02,  8.78630808e-03, -3.35063674e-02,
#         -1.01042273e-01],
#        [-1.24812932e-03,  1.09687043e-04, -6.35384035e-03,
#          2.35206584e-01,  8.91277462e-01, -4.77772820e-05,
#         -1.12290527e-04, -8.70344145e-03, -1.45707947e-01,
#         -3.10492986e-01, -1.13929092e-03, -1.52245998e-02,
#         -1.64953967e-02, -3.74670865e-03,  9.61129287e-03,
#          3.61799107e-02],
#        [ 5.51578172e-05, -2.68761727e-06,  2.16870228e-04,
#          1.27926217e-04, -4.77772820e-05,  1.00000746e+00,
#         -8.43511921e-07,  3.22461879e-04, -6.71533268e-05,
#          2.31655611e-02,  4.10475784e-05,  3.59051134e-04,
#         -4.43875693e-05,  3.75718946e-05, -7.12885284e-06,
#         -8.59159748e-05],
#        [-3.27410841e-03,  3.02049698e-01, -4.48098547e-03,
#          3.08292058e-04, -1.12290527e-04, -8.43511921e-07,
#          5.73825850e-01, -8.24659108e-03,  7.21369340e-04,
#         -1.73048668e-02,  3.00306055e-01,  7.88349353e-04,
#         -5.27826975e-04,  3.50523732e-03,  3.02821736e-04,
#          2.99307202e-04],
#        [ 4.42706601e-01, -4.31312854e-03,  8.15803062e-01,
#          1.90503255e-02, -8.70344145e-03,  3.22461879e-04,
#         -8.24659108e-03,  1.81224537e+00, -3.71770702e-02,
#          1.49574032e+00,  2.21270588e-02, -2.64359801e-01,
#          2.42684291e-02, -9.24659401e-01, -2.50739319e-03,
#          1.98293687e-03],
#        [-6.01828966e-03,  8.95864332e-04, -2.45300129e-02,
#          3.02909937e-01, -1.45707947e-01, -6.71533268e-05,
#          7.21369340e-04, -3.71770702e-02,  6.86022847e-01,
#         -1.37780206e-01, -4.51526928e-03, -3.56596026e-02,
#          3.67874964e-02, -1.50071370e-03, -3.56414526e-02,
#         -2.10716220e-01],
#        [ 4.40092680e-01, -2.47747467e-02,  9.88780298e-01,
#          8.00069322e-01, -3.10492986e-01,  2.31655611e-02,
#         -1.73048668e-02,  1.49574032e+00, -1.37780206e-01,
#          7.63253066e+02,  2.03936273e-01,  1.94933333e+00,
#          1.21187573e-01,  3.44569008e-01,  2.79172714e-01,
#         -7.21635483e-02],
#        [ 3.47452316e-03,  1.60608358e-01,  1.25919942e-02,
#          2.53432233e-03, -1.13929092e-03,  4.10475784e-05,
#          3.00306055e-01,  2.21270588e-02, -4.51526928e-03,
#          2.03936273e-01,  4.14945740e-01, -7.88397253e-03,
#          3.04322797e-03, -1.32548305e-02, -1.55908413e-04,
#          4.65965794e-04],
#        [-6.33217856e-02,  7.60594167e-04, -1.20367839e-01,
#          3.45196275e-02, -1.52245998e-02,  3.59051134e-04,
#          7.88349353e-04, -2.64359801e-01, -3.56596026e-02,
#          1.94933333e+00, -7.88397253e-03,  6.32940520e-01,
#          1.67826168e-02,  4.00658021e-01,  5.99689001e-04,
#         -1.71485827e-03],
#        [ 3.92757139e-03, -5.09218324e-04,  1.53769205e-02,
#          2.37643311e-02, -1.64953967e-02, -4.43875693e-05,
#         -5.27826975e-04,  2.42684291e-02,  3.67874964e-02,
#          1.21187573e-01,  3.04322797e-03,  1.67826168e-02,
#          7.45587404e-01, -2.41303567e-03, -6.93015664e-02,
#          2.17246725e-01],
#        [-2.25249785e-01,  2.03207051e-03, -4.16909979e-01,
#          8.78630808e-03, -3.74670865e-03,  3.75718946e-05,
#          3.50523732e-03, -9.24659401e-01, -1.50071370e-03,
#          3.44569008e-01, -1.32548305e-02,  4.00658021e-01,
#         -2.41303567e-03,  6.79588426e-01,  1.29402495e-03,
#         -1.84529085e-03],
#        [-5.20765089e-04,  1.95015914e-04, -1.42411656e-03,
#         -3.35063674e-02,  9.61129287e-03, -7.12885284e-06,
#          3.02821736e-04, -2.50739319e-03, -3.56414526e-02,
#          2.79172714e-01, -1.55908413e-04,  5.99689001e-04,
#         -6.93015664e-02,  1.29402495e-03,  9.38893721e-01,
#         -7.54809175e-02],
#        [ 3.86398138e-04,  2.94764794e-04,  9.29943710e-04,
#         -1.01042273e-01,  3.61799107e-02, -8.59159748e-05,
#          2.99307202e-04,  1.98293687e-03, -2.10716220e-01,
#         -7.21635483e-02,  4.65965794e-04, -1.71485827e-03,
#          2.17246725e-01, -1.84529085e-03, -7.54809175e-02,
#          2.41953162e-01]])
#       jac: array([ 7.45058060e-09, -1.86264515e-08,  2.53319740e-07,  5.77419996e-07,
#        -2.45869160e-07, -3.72529030e-09, -7.45058060e-08,  2.49594450e-07,
#        -1.71363354e-07,  0.00000000e+00,  1.45286322e-07,  5.62518835e-07,
#         8.94069672e-08,  2.16066837e-07,  1.75088644e-07,  2.45869160e-07])
#   message: 'Optimization terminated successfully.'
#      nfev: 648
#       nit: 26
#      njev: 36
#    status: 0
#   success: True
#         x: array([ 1.58393732e-02, -2.49973886e-01,  1.63667420e-03,  1.05652072e-07,
#        -5.60121011e-08, -8.31450846e-09, -1.04689452e-05,  1.11925927e-02,
#        -3.44650227e-08,  1.57582301e+00,  2.24997387e+00, -1.63618375e-03,
#         9.76539032e-08,  3.13580653e+00,  7.98346978e-08, -4.63781954e-08])
# actuator turned:  0.36
# gear1 orientation: [-8.31450846e-09 -5.60121011e-08  1.58393732e-02]
# gear1 position: [-2.49973886e-01  1.63667420e-03  1.05652072e-07]
# gear2 orientation: [-3.14770342e-08  7.98346978e-08  3.13580653e+00]
# gear2 position: [ 2.24997387e+00 -1.63618375e-03  9.76539032e-08]
# stick1 orientation: [0.         0.         1.57582301]
# stick1 position: [-1.04689452e-05  1.11925927e-02 -3.44650227e-08]
# ***************************************************************
# time:  0.0006470680236816406


# balanced 0.36
#       fun: 3.852457403858102
#  hess_inv: array([[ 1.26710229e+00, -1.78974952e+00, -9.95594296e-01,
#         -8.63974966e-02,  1.80800645e-02, -2.09344268e-02,
#          1.21615967e-02, -2.35893507e+00,  3.69952730e-01,
#         -3.76057919e-02,  2.83148238e+00,  1.04884964e+00,
#          1.07997292e-01, -8.41999074e-02, -5.47715274e-02,
#          1.51469972e-02],
#        [-1.78974952e+00,  3.06625952e+00,  1.73078115e+00,
#          1.04665906e-01, -1.85621212e-02,  2.20805884e-02,
#         -1.62053552e-02,  4.01513661e+00, -5.45290062e-01,
#          3.59080014e-02, -3.31859182e+00, -1.80392772e+00,
#         -1.16342125e-01,  1.22017615e-01,  7.27260841e-02,
#         -7.84239064e-03],
#        [-9.95594296e-01,  1.73078115e+00,  1.34052671e+00,
#          6.78323058e-02, -9.70498466e-03,  2.35262179e-02,
#         -1.69337459e-03,  2.50183584e+00, -3.01476109e-01,
#         -2.10026830e-03, -2.28855903e-01, -9.22135011e-01,
#         -5.02508707e-02,  5.19152435e-02,  6.98855404e-03,
#          1.39751447e-02],
#        [-8.63974966e-02,  1.04665906e-01,  6.78323058e-02,
#          3.97975032e-01, -1.73118906e-03, -3.76765783e-02,
#          1.27013371e-03,  1.32067905e-01,  2.52985207e-01,
#         -2.24951713e-02,  6.67038367e-01, -5.88557268e-02,
#          1.40865794e-01, -1.49222416e-02, -5.79338347e-03,
#          1.21374000e-02],
#        [ 1.80800645e-02, -1.85621212e-02, -9.70498466e-03,
#         -1.73118906e-03,  3.85012112e-01,  6.88346168e-02,
#          1.10154506e-02, -2.49094877e-02,  2.08335340e-03,
#          2.61411752e-01,  1.32424144e+00,  1.31318434e-02,
#          7.50987341e-03,  3.70491043e-02, -5.02969271e-02,
#         -8.50848765e-02],
#        [-2.09344268e-02,  2.20805884e-02,  2.35262179e-02,
#         -3.76765783e-02,  6.88346168e-02,  4.50033787e-01,
#         -1.11910476e-02,  2.25425838e-02, -6.18174938e-02,
#         -1.48580587e-01, -1.32107440e+00, -8.65428959e-03,
#         -7.17705508e-02, -1.18272068e-02,  5.11603283e-02,
#          4.49161720e-02],
#        [ 1.21615967e-02, -1.62053552e-02, -1.69337459e-03,
#          1.27013371e-03,  1.10154506e-02, -1.11910476e-02,
#          1.00413679e+00, -1.44746709e-02,  5.94602821e-03,
#         -1.43757917e-02,  1.40964494e+00,  5.71881018e-03,
#          2.13281700e-02, -1.73741694e-02, -1.90151807e-02,
#          2.90982932e-02],
#        [-2.35893507e+00,  4.01513661e+00,  2.50183584e+00,
#          1.32067905e-01, -2.49094877e-02,  2.25425838e-02,
#         -1.44746709e-02,  5.75944914e+00, -7.28934852e-01,
#          2.07397249e-02, -2.74249331e+00, -2.11644857e+00,
#         -1.57356706e-01,  1.40026763e-01,  6.44488960e-02,
#          8.13866727e-03],
#        [ 3.69952730e-01, -5.45290062e-01, -3.01476109e-01,
#          2.52985207e-01,  2.08335340e-03, -6.18174938e-02,
#          5.94602821e-03, -7.28934852e-01,  6.53392787e-01,
#         -4.53283787e-02,  1.88058429e+00,  3.17971479e-01,
#          3.11547005e-01, -4.87632535e-02, -2.69129917e-02,
#          2.32096659e-02],
#        [-3.76057919e-02,  3.59080014e-02, -2.10026830e-03,
#         -2.24951713e-02,  2.61411752e-01, -1.48580587e-01,
#         -1.43757917e-02,  2.07397249e-02, -4.53283787e-02,
#          7.19865376e-01, -5.71821915e+00, -6.24585657e-03,
#         -1.02848288e-01,  1.79058595e-01,  6.57237037e-02,
#         -2.95438373e-01],
#        [ 2.83148238e+00, -3.31859182e+00, -2.28855903e-01,
#          6.67038367e-01,  1.32424144e+00, -1.32107440e+00,
#          1.40964494e+00, -2.74249331e+00,  1.88058429e+00,
#         -5.71821915e+00,  1.15679687e+03,  9.85568685e-01,
#          5.65410572e+00, -3.63717507e+00, -6.45447448e+00,
#          4.95439046e+00],
#        [ 1.04884964e+00, -1.80392772e+00, -9.22135011e-01,
#         -5.88557268e-02,  1.31318434e-02, -8.65428959e-03,
#          5.71881018e-03, -2.11644857e+00,  3.17971479e-01,
#         -6.24585657e-03,  9.85568685e-01,  1.41857522e+00,
#          4.96408041e-02, -5.94021017e-02, -2.53761545e-02,
#         -6.05561110e-03],
#        [ 1.07997292e-01, -1.16342125e-01, -5.02508707e-02,
#          1.40865794e-01,  7.50987341e-03, -7.17705508e-02,
#          2.13281700e-02, -1.57356706e-01,  3.11547005e-01,
#         -1.02848288e-01,  5.65410572e+00,  4.96408041e-02,
#          4.55728499e-01, -1.08323604e-01, -9.69000790e-02,
#          6.08639009e-02],
#        [-8.41999074e-02,  1.22017615e-01,  5.19152435e-02,
#         -1.49222416e-02,  3.70491043e-02, -1.18272068e-02,
#         -1.73741694e-02,  1.40026763e-01, -4.87632535e-02,
#          1.79058595e-01, -3.63717507e+00, -5.94021017e-02,
#         -1.08323604e-01,  8.21883474e-01,  8.07789179e-02,
#          2.54669661e-01],
#        [-5.47715274e-02,  7.27260841e-02,  6.98855404e-03,
#         -5.79338347e-03, -5.02969271e-02,  5.11603283e-02,
#         -1.90151807e-02,  6.44488960e-02, -2.69129917e-02,
#          6.57237037e-02, -6.45447448e+00, -2.53761545e-02,
#         -9.69000790e-02,  8.07789179e-02,  1.08738212e+00,
#         -1.33105303e-01],
#        [ 1.51469972e-02, -7.84239064e-03,  1.39751447e-02,
#          1.21374000e-02, -8.50848765e-02,  4.49161720e-02,
#          2.90982932e-02,  8.13866727e-03,  2.32096659e-02,
#         -2.95438373e-01,  4.95439046e+00, -6.05561110e-03,
#          6.08639009e-02,  2.54669661e-01, -1.33105303e-01,
#          3.36212844e-01]])
#       jac: array([-8.94069672e-07, -6.25848770e-06,  3.33786011e-06, -1.25169754e-06,
#         1.37388706e-05, -1.06692314e-05, -2.50339508e-06,  1.25169754e-06,
#        -3.09944153e-06,  5.68032265e-05,  5.96046448e-08, -2.74181366e-06,
#         2.05636024e-06, -6.81281090e-05,  1.15036964e-05,  1.45554543e-04])
#   message: 'Desired error not necessarily achieved due to precision loss.'
#      nfev: 1182
#       nit: 47
#      njev: 65
#    status: 2
#   success: False
#         x: array([ 6.36371870e-01, -1.64580288e+00,  3.52633894e-01, -6.97528905e-01,
#         6.52683078e-06, -5.56367914e-06, -1.26362338e-06,  1.10739473e+00,
#        -1.09791646e+00,  8.98483877e-06, -5.04686042e-01,  1.64736697e+00,
#         6.97527678e-01, -5.38450087e-06,  5.73681065e-06,  1.05496418e-05])
# actuator turned:  0.36
# gear1 orientation: [-1.26395070e-06 -5.56306538e-06  6.36371869e-01]
# gear1 position: [ 3.52633894e-01 -6.97528905e-01  6.52626430e-06]
# gear2 orientation: [ 1.05479344e-05  5.73831231e-06 -1.64580288e+00]
# gear2 position: [ 1.64736697e+00  6.97527676e-01 -5.38342019e-06]
# stick1 orientation: [ 0.          0.         -0.50468614]
# stick1 position: [ 1.10739473e+00 -1.09791646e+00  8.98610276e-06]
# ***************************************************************
# time:  0.0007050037384033203


# under const
#       fun: 2.332593321616837e-13
#  hess_inv: array([[ 4.99613238e-01,  1.06493821e-03, -1.20310519e-03,
#          1.37368019e-03, -6.87574615e-04,  1.45724952e-06,
#         -7.83085221e-03,  2.47826045e-01, -9.01122704e-04,
#          2.55912039e-01],
#        [ 1.06493821e-03,  4.14326751e-01, -1.77041265e-04,
#         -2.40016256e-04,  1.20170071e-04, -3.09704496e-07,
#          3.61716763e-01, -1.66986623e-04,  1.57457343e-04,
#          4.12464542e-02],
#        [-1.20310519e-03, -1.77041265e-04,  4.96124743e-01,
#          4.40767318e-03, -2.20691091e-03,  5.89183372e-06,
#         -6.96227326e-04,  4.93570229e-01, -2.89158224e-03,
#         -7.83320604e-01],
#        [ 1.37368019e-03, -2.40016256e-04,  4.40767318e-03,
#          5.02235762e-01,  2.48921423e-01,  3.63588996e-07,
#          7.95551404e-05,  7.31342029e-03,  3.26470664e-01,
#          8.91540236e-01],
#        [-6.87574615e-04,  1.20170071e-04, -2.20691091e-03,
#          2.48921423e-01,  8.75519634e-01, -1.81699900e-07,
#         -3.98380591e-05, -3.66168979e-03, -1.63261110e-01,
#         -4.45799093e-01],
#        [ 1.45724952e-06, -3.09704496e-07,  5.89183372e-06,
#          3.63588996e-07, -1.81699900e-07,  1.00000000e+00,
#          1.18199853e-07,  9.58926223e-06, -2.38435140e-07,
#         -9.26416470e-05],
#        [-7.83085221e-03,  3.61716763e-01, -6.96227326e-04,
#          7.95551404e-05, -3.98380591e-05,  1.18199853e-07,
#          7.76934259e-01, -5.65828665e-03, -5.21921380e-05,
#         -2.58333480e-02],
#        [ 2.47826045e-01, -1.66986623e-04,  4.93570229e-01,
#          7.31342029e-03, -3.66168979e-03,  9.58926223e-06,
#         -5.65828665e-03,  1.11415515e+00, -4.79781890e-03,
#         -1.04988721e+00],
#        [-9.01122704e-04,  1.57457343e-04, -2.89158224e-03,
#          3.26470664e-01, -1.63261110e-01, -2.38435140e-07,
#         -5.21921380e-05, -4.79781890e-03,  7.85876353e-01,
#         -5.84727615e-01],
#        [ 2.55912039e-01,  4.12464542e-02, -7.83320604e-01,
#          8.91540236e-01, -4.45799093e-01, -9.26416470e-05,
#         -2.58333480e-02, -1.04988721e+00, -5.84727615e-01,
#          1.48350431e+03]])
#       jac: array([ 4.92475691e-12, -2.88160850e-08,  1.07703259e-07, -7.17546405e-07,
#         3.58682867e-07,  3.17988115e-10,  1.70037816e-08,  1.09234445e-07,
#         4.70582040e-07,  1.33424668e-08])
#   message: 'Optimization terminated successfully.'
#      nfev: 300
#       nit: 20
#      njev: 25
#    status: 0
#   success: True
#         x: array([ 3.76991327e-02, -2.82578935e-08,  8.61171103e-08, -1.45833924e-07,
#         5.60886328e-08, -7.29163495e-09,  4.99644709e-01,  1.88452351e-02,
#         5.39621992e-08,  3.77210257e-02])
# actuator turned:  1.8
# gear1 orientation: [-7.29163495e-09  5.60886328e-08  3.76991327e-02]
# gear1 position: [-2.82578935e-08  8.61171103e-08 -1.45833924e-07]
# gear2 orientation: [0. 0. 0.]
# gear2 position: [2 0 0]
# stick1 orientation: [0.         0.         0.03772104]
# stick1 position: [4.99644709e-01 1.88452351e-02 5.39621992e-08]
# ***************************************************************
# time:  0.0006048679351806641

