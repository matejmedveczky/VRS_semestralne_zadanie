# Battery branch



1. POPIS MODULU

   - Tento modul zabezpečuje monitorovanie stavu napájacej batérie mobilného robota. Jeho úlohou je priebežne sledovať napätie
     batérie, vyhodnocovať jej stav a na základe toho informovať nadradený riadiaci systém a obmedzovať činnosť robota.



---



2. FUNKCIONALITA

   - Meranie napätia batérie pomocou ADC (cez voltage divider)

  - Vyhodnotenie stavu batérie na základe nameraného napätia

  - Detekcia nízkeho napätia batérie:

	- napätie < 10.5 V → varovný stav

  - Odosielanie informácií o stave batérie do nadradeného systému (Raspberry Pi)

	- zobrazenie stavu batérie v riadiacom rozhraní

  - Obmedzenie funkcií mobilného robota na základe stavu batérie:
	- zníženie výkonu
	- deaktivácia vybraných funkcií pri kritickom napätí

