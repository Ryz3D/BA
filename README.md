# Bachelorarbeit

## Repository
In diesem Repository sind alle zugehörigen Daten der Bachelorarbeit von Mirco Heitmann zu finden.
Die Ordnerstruktur ist wie folgt aufgebaut:
| Ordner    | Inhalt |
| --------- | ------------- |
| Hardware  | Schaltpläne, Platinenlayout und Schaltungssimulationen |
| Messungen | Messdaten zu durchgeführten Tests |
| Python    | Python-Skripts zur Datenauswertung und Simulation |
| STM32     | STM32CubeIDE-Projektdateien |

## Zusammenfassung
Dieser Bericht behandelt die Entwicklung eines mikrocontrollerbasierten Systems zur Messung von Beschleunigungen mithilfe eines MEMS-Sensors über eine digitale Schnittstelle sowie mit Piezoelementen und einer dazugehörigen analogen Schaltung als Messfilter und -verstärker, außerdem mit synchronisierter Ortung per GNSS. Entwickelt wurde das System für Vibrationsmessungen am Eisenbahnrad zur Verschleißerkennung als Teil des THM-Projekts VeRa. Beschrieben wird die Auswahl und Integration verschiedener Entwicklungsboards, die Entwicklung einer projektspezifischen Platine und der entsprechenden Software, sowie einige Tests des Messsystems (Labormessungen und eine Messung im KFZ), um Unterschiede zwischen piezoelektrischen und auf kapazitiven MEMS basierenden Beschleunigungssensoren aufzuzeigen.

## Abstract
This paper presents the development of a uC-based acceleration measurement system utilizing MEMS sensors in conjuction with piezoelectric elements, as well as synchronized GNSS positioning. The focus of this paper is comparing the results of MEMS acceleration sensors to piezoelectric acceleration sensors. The MEMS sensor is directly connected via a digital interface, while the piezoeletric sensors require analog filtering and amplification. The aim of the project is to measure vibrations near train wheels in order to gather data on wheel wear as part of the THM university project VeRa. This paper covers the integration of development boards, designing a custom PCB, software development and system validation through test results of lab tests and real-world acceleration measurements taken from a car.
