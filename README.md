# amr_domaci_2

Ovaj repozitorijum sadrži radni prostor, kao i pakete `amr_domaci_2` i `amr_domaci_2_interfejsi` dobijene pri izradi 2. domaćeg zadatka na predmetu Autonomni mobilni roboti.

## Preduslovi

* Koristite Ubuntu 22.04 ili Ubuntu 24.04
* Instaliran ROS2; ukoliko to nije učinjeno do sad, pratiti uputstva za instalaciju [ovde](https://docs.ros.org/en/kilted/Installation.html) (ROS2 Kilted Kaiju) ako koristite Ubuntu 24.04, ili [ovde](https://docs.ros.org/en/humble/Installation.html) ako koristite Ubuntu 22.04
* Instaliran [git](https://git-scm.com); git se može preuzeti kucanjem narednog koda u linux terminal:
  ```
  apt-get install git
  ```
* Instaliran [xterm](https://xterm.dev) koji se može preuzeti kucanjem narednog koda u linux terminal:
  ```
   sudo apt install xterm
  ```

## Priprema za korišćenje paketa

S obzirom da repozitorijum obuhvata ceo radni prostor, nije pozeljno čuvati ga unutar već postojećeg radnog prostora. U linux terminalu možete napisati sledeće:

```
cd ~
git clone https://github.com/Bigulac/amr_domaci_2.git
```

Ostalo je još da se paket izbilduje. U linux terminalu iskucati sledeće:

```
cd ~/amr_domaci_2
colcon build --symlink-install
```

Opcija `--symlink-install` dozvoljava da se mogu menjati postojeće .py skripte bez potrebe ponovnog bildovanja paketa.
