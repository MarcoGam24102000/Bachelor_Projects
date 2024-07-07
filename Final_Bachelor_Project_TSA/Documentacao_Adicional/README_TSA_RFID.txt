
Pensou-se na possibilidade de existir ainda alguma robustez na deteção do atleta.

Deste modo, dado que o processador utilizado na linha de chegada, não possui pinos suficientes:
1) Foi planeada a adição de um outro processador (ESP32) com uma RFID (a comunicar por UART) e um LED, que, quando a RFID detetasse o atleta, serie enviada a confirmação
por ESP-NOW, para o processador presente na linha de chegada (sendo que este processador (bem como o circuito constituinte) deverá também de ficar na linha de chegada).
Desta forma, o atleta deveria de ser portador de um Tag (a colocar, eventualmente nas sapatilhas), sendo que a deteção do atleta é feita recorrendo à câmara presente na linha de chegada
bem como à identificação por radiofrequência (através do módulo RFID cuja frequência é de 125 kHz).
2) Este módulo adicional deverá ou não de ficar numa PCB à parte - tal dependerá de como for mais acessível, para os atletas, o transporte dos dispostivos.