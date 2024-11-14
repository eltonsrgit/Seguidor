#ifndef __Teste_H
#define __Teste_H

#define CH1 2
#define CH2 3


unsigned long Contador1; // Contadores
unsigned long Contador2; // Para guardar a leitura dos sensores
int Numero_Dentes = 6;   // Referentes as engrenagens

// Função para calcular a velocidade angular média em um intervalo de tempo
float pega_velocidade_cru(unsigned long t_min)
{
  static unsigned long t0 = 0; // Variável para armazenar o tempo
  static float velocidade_angular = 0; // Explícito

  // Verifica se o tempo mínimo especificado passou
  if ((millis() - t0) >= t_min) { 
    
    // Calcula a média das leituras dos contadores
    float media_leituras = (Contador1 + Contador2) / 2.0;
    float rotacoes = media_leituras / (Numero_Dentes * 2);

    // Calcula a velocidade angular em rotações por minuto
    velocidade_angular = 1000 * rotacoes / t_min;
    
    // Reinicia os contadores e atualiza o tempo
    Contador1 = Contador2 = 0;
    t0 = millis();
  }
  
  return velocidade_angular;
}

// Tempo em milisegundos para pegar a velocidades... No Caso 100 milisegundos
#define pega_velocidade() pega_velocidade_cru(100)

// Função de interrupção para contar pulsos no canal 1
void Contador_Pulso1()
{
  Contador1++;
}

// Função de interrupção para contar pulsos no canal 2
void Contador_Pulso2()
{
  Contador2++;
}

#endif