# Trabajo Final: Modelado y Simulación del Robot FANUC M-10iD/12

**Institución:** Facultad de Ingeniería, Universidad Nacional de Cuyo (UNCUYO)  
**Asignatura:** Robótica 1  

Este repositorio contiene todo el desarrollo técnico, analítico y el código fuente correspondiente al Trabajo Final de la materia Robótica 1. El proyecto aborda el modelado cinemático completo y la planificación de tareas de un manipulador industrial **FANUC M-10iD/12** equipado con una antorcha de soldadura PSF 315M, culminando en la simulación de una tarea de soldadura de trayectoria rectilínea.

## 👥 Equipo de Trabajo
- Albanez Luciano - 13185
- Jofre Joaquin - 13473
- Papetti Mario - 11807
- Torres Ignacio - 13621

---

## 🎯 Evolución y Alcance del Proyecto

El desarrollo del Trabajo Final integra los conocimientos adquiridos a lo largo de los distintos Trabajos Prácticos de la materia, aplicados específicamente a este modelo de FANUC:

1. **Parámetros Estructurales y D-H:** Definición del esqueleto del robot utilizando la convención de Denavit-Hartenberg, asignación de sistemas de coordenadas, determinación de límites articulares (`qlim`) y modelado del efector final (TCP).
2. **Cinemática Directa e Inversa:** Implementación de algoritmos para determinar la posición espacial a partir de los motores, y el cálculo de la configuración articular necesaria para alcanzar una pose requerida.
3. **Análisis Diferencial (Jacobiano):** Cálculo de la matriz Jacobiana para mapear velocidades articulares a velocidades cartesianas, y análisis de singularidades en el espacio de trabajo del robot.
4. **Planificación de Trayectorias (Tarea de Soldadura):** Generación de trayectorias evaluando la interpolación en el espacio articular (curvas suaves de motores) versus la interpolación en el espacio cartesiano (movimiento rectilíneo estricto del TCP para el cordón de soldadura).

---

## 📂 Estructura del Repositorio

```text
📦 TF_Robotica1_FANUC
 ┣ 📂 documentacion/
 ┃ ┣ 📜 FANUC_M10iD-12_MDS-03823-EN.pdf (Hoja de datos del robot)
 ┃ ┣ 📜 TOOL_torcha_soldadura_501032-502.pdf (Manual de la herramienta)
 ┃ ┗ 📜 Informe_Final_Grupo.pdf (Reporte completo del TF)
 ┣ 📂 codigo_matlab/
 ┃ ┣ 📜 robot_def.m (Definición D-H, TCP, límites y offsets base)
 ┃ ┣ 📜 cinematica_analisis.m (Validación de cinemática directa e inversa)
 ┃ ┣ 📜 jacobiano_singularidades.m (Cálculo del Jacobiano y determinantes)
 ┃ ┗ 📜 robot_traj.m (Simulación comparativa de trayectorias de soldadura)
 ┣ 📂 multimedia/
 ┃ ┣ 🖼️ perfiles_cartesianos.png
 ┃ ┣ 🖼️ perfiles_articulares.png
 ┃ ┗ 🖼️ simulacion_espacial_3D.pnghttps://grabcad.com/library/fanuc-m-10id-12-1

 ┗ 📜 README.md
```

---
*Repositorio creado para fines académicos. Robótica 1 - FI UNCUYO.*
