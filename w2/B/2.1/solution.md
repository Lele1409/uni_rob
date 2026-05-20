## Lösung Aufgabe B2.1, Differenzialantrieb

Genutzte Folien: Differenzialantrieb (2) [Folie 130], Praktische Approximationen [Folie 136], Beispiel: Rechnung nach Approximation [Folie 137/138]

### Gegebene Größen
- Startpose: $(x_0, z_0, \theta_0)^T = (0\,\text{cm},\, 0\,\text{cm},\, 0°)$
- Achsabstand: $b = 30\,\text{cm}$
- Kinematik (aus Aufgabenblatt):

$$\Delta d = \frac{s_l + s_r}{2}, \qquad \Delta\theta = \frac{s_r - s_l}{b}$$

Aus Folie 136 nutzen wir die Approximationsformel:

$$\begin{pmatrix} x_n \\ z_n \\ \theta_n \end{pmatrix} = \begin{pmatrix} x_{n-1} \\ z_{n-1} \\ \theta_{n-1} \end{pmatrix} + \begin{pmatrix} \Delta d \cdot \sin\!\left(\theta_{n-1} + \frac{\Delta\theta}{2}\right) \\ \Delta d \cdot \cos\!\left(\theta_{n-1} + \frac{\Delta\theta}{2}\right) \\ \Delta\theta \end{pmatrix}$$

Im Grunde haben wir: $NeuePos \approx AltePos + Distanz \cdot PosAufDemEinheitskreis$, wobei der Ursprung dieses Einheitskreis an der alten Position ist. 

### Teil (a), Berechnung der Posen

#### Zeitschritt t = 1 ($s_l = 10\,\text{cm},\; s_r = 6\,\text{cm}$)

$$\Delta d = \frac{10 + 6}{2} = 8\,\text{cm}$$

$$\Delta\theta = \frac{6 - 10}{30} = -\frac{4}{30} = -\frac{2}{15}\,\text{rad} \approx -7{,}64°$$

Mittlerer Winkel: $\theta_{mid} = 0 + \frac{-2/15}{2} = -\frac{1}{15}\,\text{rad} \approx -3{,}82°$

$$x_1 = 0 + 8 \cdot \sin\!\left(-\tfrac{1}{15}\right) \approx 8 \cdot (-0{,}0666) \approx -0{,}53\,\text{cm}$$
$$z_1 = 0 + 8 \cdot \cos\!\left(-\tfrac{1}{15}\right) \approx 8 \cdot 0{,}9978 \approx 7{,}98\,\text{cm}$$
$$\theta_1 = -\tfrac{2}{15}\,\text{rad} \approx -7{,}64°$$

$$\boxed{(x_1, z_1, \theta_1)^T \approx (-0{,}53\,\text{cm},\; 7{,}98\,\text{cm},\; -7{,}64°)}$$

#### Zeitschritt t = 2 ($s_l = 5\,\text{cm},\; s_r = 7\,\text{cm}$)

$$\Delta d = \frac{5 + 7}{2} = 6\,\text{cm}$$

$$\Delta\theta = \frac{7 - 5}{30} = \frac{2}{30} = \frac{1}{15}\,\text{rad} \approx +3{,}82°$$

Mittlerer Winkel: $\theta_{mid} = -\frac{2}{15} + \frac{1}{30} = -\frac{1}{10}\,\text{rad} \approx -5{,}73°$

$$x_2 = -0{,}53 + 6 \cdot \sin\!\left(-\tfrac{1}{10}\right) \approx -0{,}53 + 6 \cdot (-0{,}0998) \approx -1{,}13\,\text{cm}$$
$$z_2 = 7{,}98 + 6 \cdot \cos\!\left(-\tfrac{1}{10}\right) \approx 7{,}98 + 6 \cdot 0{,}9950 \approx 13{,}95\,\text{cm}$$
$$\theta_2 = -\tfrac{2}{15} + \tfrac{1}{15} = -\tfrac{1}{15}\,\text{rad} \approx -3{,}82°$$

$$\boxed{(x_2, z_2, \theta_2)^T \approx (-1{,}13\,\text{cm},\; 13{,}95\,\text{cm},\; -3{,}82°)}$$

### Teil (b), Welcher Fehler wirkt sich langfristig stärker aus?

Der Rotationsfehler wirkt sich langfristig stärker aus.

Wie Folie 135 zeigt, gehen in die Positionsberechnung die Terme $\sin(\theta)$ und $\cos(\theta)$ ein. Ein kleiner, konstanter Winkelfehler $\Delta\theta$ bewirkt, dass die Fahrtrichtung dauerhaft falsch ist, der Roboter fährt eine Kurve statt geradeaus. Dadurch wächst der Positionsfehler unbegrenzt an (Zitat Folie 125: „Gesamtfehler der Schätzung durch die Integration praktisch unbeschränkt").

Ein reiner Translationsfehler hingegen addiert sich zwar auf, ändert aber nicht die Richtung der weiteren Bewegung, der Fehler wächst damit lediglich linear, nicht wie beim Rotationsfehler.
