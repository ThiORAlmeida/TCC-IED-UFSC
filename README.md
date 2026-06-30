# Relé de Proteção Inteligente para Transformadores

Repositório do Trabalho de Conclusão de Curso em Engenharia Eletrônica, contendo os códigos e arquivos de apoio do protótipo didático de um **relé de proteção inteligente para transformadores**.

## Sobre o Projeto

O projeto consiste em um sistema experimental baseado em dois microcontroladores ESP32, sensores de tensão e corrente, interface local, saída de trip e comunicação digital.

A arquitetura foi dividida em dois módulos:

* **ESP32 1:** aquisição dos sinais de tensão, corrente e temperatura.
* **ESP32 2:** processamento das proteções, lógica de trip, LEDs, relé de saída e comunicação Modbus TCP/IP.

## Funções Implementadas

O protótipo contempla as seguintes funções ANSI:

| Código | Função                    |
| ------ | ------------------------- |
| 50     | Sobrecorrente instantânea |
| 51     | Sobrecorrente temporizada |
| 59     | Sobretensão               |
| 27     | Subtensão                 |
| 26     | Proteção térmica          |
| 81U    | Subfrequência             |
| 81O    | Sobrefrequência           |
| 24     | Sobreexcitação            |

## Componentes Principais

* 2 ESP32;
* Sensores de tensão ZMPT101B;
* Sensores de corrente ACS712;
* Módulo Ethernet W5500;
* Fonte 5 V;
* Display;
* Barra de LEDs;
* Relé de saída;
* Filtros analógicos com resistores e capacitores.

## Comunicação

A comunicação entre os ESP32 é feita por serial.
A comunicação externa é realizada via Ethernet, utilizando o módulo W5500 com Modbus TCP/IP.

## Observação

Este projeto possui finalidade **didática, acadêmica e experimental**.
O protótipo não substitui relés comerciais e não deve ser aplicado diretamente em instalações reais sem adequações, ensaios e certificações.

## Autor

**Thiago Oliveira Rodrigues Almeida**
Engenharia Eletrônica – UFSC
