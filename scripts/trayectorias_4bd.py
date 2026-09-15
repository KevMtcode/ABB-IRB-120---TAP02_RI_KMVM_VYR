import numpy as np
import matplotlib.pyplot as plt
import csv

from perfiles import generar_tramo

PRE_PICK = np.array([0.356,-0.397,0.250])
PICK = np.array([0.351,-0.411,0.045])
PRE_PLACE = np.array([0.402,0.360,0.411])
PLACE = np.array([0.413,0.362,0.217])

Q_4B = [
    0.248,
    0.650,
   -0.299,
    0.654
]

Q_4D = [
    0.284,
    0.634,
   -0.261,
    0.670
]

ida = "ida"
vuelta = "vuelta"


def guardar(nombre, waypoints, q):

    with open(
        "scripts/waypoints/" + nombre + ".csv",
        "w",
        newline=""
    ) as archivo:

        writer = csv.writer(archivo)

        writer.writerow([
            "tiempo",
            "x", "y", "z",
            "qx", "qy", "qz", "qw"
        ])

        for punto in waypoints:

            writer.writerow([
                punto[0],
                punto[1],
                punto[2],
                punto[3],
                q[0],
                q[1],
                q[2],
                q[3]
            ])


def comparar(tramo, sentido):

    if tramo == "4B":

        orientacion = Q_4B

        if sentido == ida:
            inicio = PRE_PICK
            final = PICK

        elif sentido == vuelta:
            inicio = PICK
            final = PRE_PICK


    elif tramo == "4D":

        orientacion = Q_4D

        if sentido == ida:
            inicio = PRE_PLACE
            final = PLACE

        elif sentido == vuelta:
            inicio = PLACE
            final = PRE_PLACE

    else:
        print("Tramo no valido")
        return


    if sentido == ida:

        vmax = 0.200
        amax = 0.300
        color = "red"

    elif sentido == vuelta:

        vmax = 0.100
        amax = 0.020
        color = "blue"

    else:
        print("Sentido no valido")
        return


    for perfil in ["cubico", "quintico"]:

        datos = generar_tramo(
            inicio,
            final,
            vmax,
            amax,
            perfil
        )

        t, posicion, velocidad, aceleracion, waypoints, T = datos

        guardar(
            tramo + "_" + perfil + "_" + sentido,
            waypoints,
            orientacion
        )

        vel_max = max(abs(velocidad))
        acc_max = max(abs(aceleracion))

        fig, ax = plt.subplots(
            3,
            1,
            figsize=(8, 9)
        )

        fig.suptitle(
            tramo + " - " +
            perfil.capitalize() + " - " +
            sentido.capitalize()
            + f"\nTiempo = {T:.3f} s"
            + f" | Vmax = {vel_max:.3f} m/s"
            + f" | Amax = {acc_max:.3f} m/s²"
        )

        ax[0].plot(t, posicion, color=color)
        ax[0].set_ylabel("Posicion [m]")
        ax[0].set_title("Posicion")
        ax[0].grid()

        ax[1].plot(t, velocidad, color=color)
        ax[1].axhline(vmax, linestyle="--")
        ax[1].set_ylabel("Velocidad [m/s]")
        ax[1].set_title("Velocidad")
        ax[1].grid()

        ax[2].plot(t, aceleracion, color=color)
        ax[2].axhline(amax, linestyle="--")
        ax[2].axhline(-amax, linestyle="--")
        ax[2].set_ylabel("Aceleracion [m/s²]")
        ax[2].set_xlabel("Tiempo [s]")
        ax[2].set_title("Aceleracion")
        ax[2].grid()

        plt.tight_layout()


comparar("4B", ida)
comparar("4B", vuelta)

comparar("4D", ida)
comparar("4D", vuelta)

plt.show()
