---
name: project-assignment3
description: Portfolio opdracht 3 — Path-Planning, Navigation & Control voor Duckiebot in een doolhof
metadata:
  type: project
---

Doel: Duckiebot autonoom van start naar doel laten rijden via de kortste route, met obstakelvermijding, lokalisatie en PID-regeling.

**Vier taken:**

1. **Taak 1 – Graafgebaseerde padplanning**
   - Kortste route berekenen met een graafrepresentatie van de Duckietown-kaart (nodes = kruispunten/tegels, edges = verbindingen met afstanden in tegels)
   - Waarschijnlijk Dijkstra of A*

2. **Taak 2 – Gekalibreerde lokalisatie**
   - Huidige node + volgende node + resterend pad bijhouden
   - Technieken: odometry (wielencoders), AprilTags, kruispunttelling, lanedetectie

3. **Taak 3 – Navigatie**
   - Geplande route uitvoeren
   - Op kruispunten: rechtdoor / linksaf / rechtsaf
   - Statische obstakels (duckies) vermijden

4. **Taak 4 – Controle met feedback**
   - Feedbackregelaar (PID of RL)
   - Minimaliseer rijstrookafwijking, verminder slingeren, handhaaf gewenste snelheid

**Deliverables:**
- Nette ROS packages in `/packages/`
- Technisch verslag (2–3 pagina's): architectuur, implementatie, keuzes, beperkingen
- Live demo (kaart wordt vóór demonstratiedag toegestuurd; duckie-posities zijn willekeurig)

**Technische context:**
- ROS framework, Duckietown `daffy` distro, base image `dt-core`
- Repo is een lege Duckietown template; packages komen in `/packages/`

**Voorbeeld kaart:**
```
A ── B ── C ── D
│       │   │
E   F ── G   H
│   │       │
S ── I ── J ── T
```
S = Start, T = Doel. Kortste pad: S→I→J→T

**Why:** Schoolopdracht voor autonoom rijden met Duckiebot.
**How to apply:** Bij elk implementatieonderdeel rekening houden met de vier taken en de deliverable-eisen. Kaart/duckie-posities zijn pas op demonstratiedag bekend — code moet generiek zijn.
