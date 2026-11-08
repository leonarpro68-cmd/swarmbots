# Generador offline de escenarios — supermarket

No modifica ni ejecuta archivos de la misión validada de `tugbot_warehouse`.

## Uso

```bash
python3 generar_escenario.py --seed 2026 --output-dir resultados/supermarket/seed_2026
```

Requiere Python 3 y `matplotlib` para crear el PNG. El grafo y los obstáculos predeterminados se leen de `config/maps/supermarket/grafo.json` y `config/maps/supermarket/obstaculos.json`; no hay copias locales junto al script.

## Reglas

- Robots (3) y depósitos (3): exclusivamente cualquiera de los 588 vértices libres del grafo, sin filtro de seis vecinos.
- Basuritas (6, dos por color): coordenadas continuas en `[-13.75,13.75] × [-23.75,22.50]`, mínimo 0.75 m desde el **centro** al contorno de cualquiera de los 14 rectángulos.
- Separación euclidiana mínima de 5.0 m entre los 12 elementos, de cualquier tipo.
- Cada basurita cuenta con un nodo de aproximación conectado mediante un segmento que respeta 0.75 m frente a obstáculos. Esto es un criterio preliminar, no una validación de agarre.
- La semilla permite repetir exactamente el escenario.

## Salidas

- `scenario.json`: coordenadas de los 12 elementos, índices `(i,j)` para robots y depósitos, posición continua y nodo de aproximación para basuritas.
- `escenario.png`: vista general.
- `validacion.json`: banderas de validación y sus límites.

**No se ha comprobado que la misión A*/CBS/MILP/Bézier sea factible ni que las maniobras físicas funcionen en Gazebo.** La figura usa una **aproximación visual** rectangular a la banda de exclusión; la comprobación numérica es respecto a la distancia euclidiana exacta a los rectángulos.
