#ifndef __Teste2_H
#define __Teste2_H

#define CH3 18
#define CH4 19

unsigned long Contador3; // Contadores
unsigned long Contador4; // Para guardar a leitura dos sensores
int Numero_Dentes2 = 6;   // Referentes as engrenagens

// Função para calcular a velocidade angular média em um intervalo de tempo
float pega_velocidade_cru2(unsigned long t_min2)
{
  static unsigned long t1 = 0; // Variável para armazenar o tempo
  static float velocidade_angular2 = 0; // Explícito

  // Verifica se o tempo mínimo especificado passou
  if ((millis() - t1) >= t_min2) { 
    
    // Calcula a média das leituras dos contadores
    float media_leituras2 = (Contador3 + Contador4) / 2.0;
    float rotacoes2 = media_leituras2 / (Numero_Dentes2 * 2);

    // Calcula a velocidade angular em rotações por minuto
    velocidade_angular2 = 1000 * rotacoes2 / t_min2;
    
    // Reinicia os contadores e atualiza o tempo
    Contador3 = Contador4 = 0;
    t1 = millis();
  }
  
  return velocidade_angular2;
}

// Tempo em milisegundos para pegar a velocidades... No Caso 100 milisegundos
#define pega_velocidade2() pega_velocidade_cru2(100)

// Função de interrupção para contar pulsos no canal 3
void Contador_Pulso3()
{
  Contador3++;
}

// Função de interrupção para contar pulsos no canal 4
void Contador_Pulso4()
{
  Contador4++;
}

#endif