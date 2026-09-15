import numpy as np

def cubico(u):
    s = 3*u**2 - 2*u**3
    ds = 6*u - 6*u**2
    dds = 6 - 12*u
    return s, ds, dds

def quintico(u):
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = 30*u**2 - 60*u**3 + 30*u**4
    dds = 60*u - 180*u**2 + 120*u**3
    return s, ds, dds


def generar_tramo(inicio, final, vmax, amax, perfil):
    distancia = np.linalg.norm(final - inicio)
    if perfil == "cubico":
        funcion = cubico
        T_vel = 1.5 * distancia / vmax
        T_acc = np.sqrt(6 * distancia / amax)

    elif perfil == "quintico":
        funcion = quintico
        T_vel = 1.875 * distancia / vmax
        T_acc = np.sqrt(5.7735 * distancia / amax)

    else:
        raise ValueError("Perfil debe ser 'cubico' o 'quintico'")

    T = max(T_vel, T_acc)

    t = np.linspace(0, T, 100)
    u = t / T

    s, ds, dds = funcion(u)

    posicion = distancia * s
    velocidad = distancia * ds / T
    aceleracion = distancia * dds / T**2

    # inicio + 4 puntos intermedios + final
    tiempos_wp = np.linspace(0, T, 6)
    u_wp = tiempos_wp / T

    s_wp, _, _ = funcion(u_wp)

    waypoints = []

    for tiempo, avance in zip(tiempos_wp, s_wp):

        xyz = inicio + avance * (final - inicio)

        waypoints.append([
            tiempo,
            xyz[0],
            xyz[1],
            xyz[2]
        ])

    return (t,posicion,velocidad,aceleracion,waypoints,T)
