# Seguidor de linha para o laboratório de Servomec

O seguidor de linha é um robô com o objetivo de completar o percurso de uma linha em uma pista com trechos retos, curvas e desvios.
É montado por uma carcaça impressa, e possui para o seu funcionamento os seguintes componentes eletrônicos e mecânicos:

# Arduino MEGA (Robocore)
  Com o Arduino Mega, podemos ter o controle do seguidor e é nele onde fica armazenado o código (em C++) e é ele quem vai receber sinal de sensores e controlar o robô para que atenda nosso objetivo.  
  É alimentado por 5 volts pela ponte H L298N.

  # Módulo Sensor de Linha 5 Canais Reflexivo Tcrt5000

  Esses sensores digitais identificam a cor da pista, de modo que o robô possa identificar onde se encontra dependendo de quais sensor estão ativos e quais estão desativados, este sensor enviará um sinal alto (HIGH) se identificar a cor branca e baixo (LOW) se a cor identificada for preta.  
  Alimentação: 5 volts;  
  Portas digitais escolhidas no Arduino Mega (do primeiro em diante respectivamente): 12, 11, 10, 9 e 8.  
  
  # Driver de motors Ponte H L298N

  A ponte H L298N recebe o sinal PWM do Arduino e o manda para os motores e com que o Arduino não precise ceder potência para isso, já que a ponte H utiliza a potência da bateria para fazer essa comunicação: sinal do arduino para motores.  
  Alimentação: 7,4 à 8,4 de uma bateria de LiPo;  
  Pinos digitais e PWM escolhidos no Arduino:  
  Motor direito: 4 e 5; (conectado em IN1 e IN2 da ponte H, respectivamente)  
  Mortor esquerdo: 6 e 7; (conectado em IN3 e IN4 da ponte H, respectivamente)  
  Lembrando que é preciso que pelo menos um dos pinos de cada motor seja PWM (sinalizado por "~" na placa Arduino) pois o controle depende disso.
  
  
  
