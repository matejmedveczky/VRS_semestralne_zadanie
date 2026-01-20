# Battery branch



### 1. POPIS MODULU

   - Tento modul zabezpečuje monitorovanie stavu napájacej batérie mobilného robota. Jeho úlohou je priebežne sledovať napätie
     batérie, vyhodnocovať jej stav a na základe toho informovať nadradený riadiaci systém a obmedzovať činnosť robota.



---



### 2. FUNKCIONALITA

  - Meranie napätia batérie pomocou ADC (cez voltage divider)

  - Vyhodnotenie stavu batérie na základe nameraného napätia

  - Detekcia nízkeho napätia batérie:

	- napätie < 10.5 V → varovný stav

  - Odosielanie informácií o stave batérie do nadradeného systému (Raspberry Pi)

	- zobrazenie stavu batérie v riadiacom rozhraní

  - Obmedzenie funkcií mobilného robota na základe stavu batérie:
	- zníženie výkonu
	- deaktivácia vybraných funkcií pri kritickom napätí

## Návod na použitie
### Klonovanie
```bash
cd lokalny priecinok
git clone https://github.com/MatejMedveczky/VRS_semestralne_zadanie.git
cd VRS_semestralne_zadanie
git switch -c VHODNY_NAZOV_BRANCH
```
### Prvý commit
```bash
git add .
git commit -m "POPIS ZMENY"
git push -u origin <VHODNY_NAZOV_BRANCH>
```
### Dalšie commity
Skontroluj či si v správnej branchi
```bash
git add .
git commit -m "POPIS ZMENY"
git push
```


