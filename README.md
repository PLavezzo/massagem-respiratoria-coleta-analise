# Sistema de Coleta e Análise - Massagem Respiratória

Sistema para coleta e análise de dados de sensores de pressão MPX5050DP via ESP32, utilizado em pesquisas de fisioterapia respiratória.

## Componentes

### 1. Coleta.py

Programa principal para coleta de dados em tempo real.

**Funcionalidades:**

- Conexão serial com ESP32 (250000 baud)
- Visualização em tempo real de dois canais de pressão
- Calibração interativa de 2 pontos com cache persistente
- Detecção automática de picos respiratórios
- Filtro passa-baixas RC de 1ª ordem (60 Hz)
- Aquecimento de 5 segundos antes da coleta
- Exportação para CSV com timestamp

**Como usar:**

```bash
python Coleta.py
```

### 2. Analise.py

Ferramenta para análise pós-coleta de dados.

**Funcionalidades:**

- Carregamento de arquivos CSV/XLSX
- Corte temporal de sinais
- Detecção avançada de picos com parâmetros ajustáveis
- Análise FFT com identificação de frequência dominante
- Exportação de análises completas para Excel

**Como usar:**

```bash
python Analise.py
```

## Hardware

**Sensor:** MPX5050DP (pressão diferencial, 0-50 kPa)
**Microcontrolador:** ESP32
**Divisor de tensão:** R1=10kΩ, R2=18kΩ (5V → 3.3V)
**ADC:** 12 bits, 3.3V full scale

## Protocolo Serial

- Marcador: `0xFF`
- Estrutura: `[0xFF][VX_L][VX_H][VY_L][VY_H][T0][T1][T2][T3]`
- Total: 9 bytes por pacote
- Taxa: 250000 baud

## Calibração

Sistema de calibração de 2 pontos integrado ao Coleta.py:

1. Conectar ESP32 e clicar em "Calibração"
2. Aplicar pressão baixa (~30 mmHg) e capturar ponto 1
3. Aplicar pressão alta (~60 mmHg) e capturar ponto 2
4. Fatores salvos automaticamente em `.cache_calibracao.json`

Fórmula: `P_real = offset + gain × P_teórica`

## Instalação

```bash
pip install -r requirements.txt
```

## Criar Executáveis

Para Windows/Mac/Linux:

```bash
pyinstaller --onefile --windowed Coleta.py
pyinstaller --onefile --windowed Analise.py
```

Os executáveis ficarão em `dist/`

## Estrutura de Dados

Arquivo CSV de saída:

- `Tempo (s)`: Timestamp relativo
- `Valor X`: ADC bruto canal X
- `Valor Y`: ADC bruto canal Y
- `Pressão X (mmHg)`: Pressão calibrada canal X
- `Pressão Y (mmHg)`: Pressão calibrada canal Y

## Requisitos do Sistema

- Python 3.8+
- Porta serial disponível
- Sistema operacional: Windows, macOS ou Linux

## Autores

Desenvolvido para pesquisa em fisioterapia respiratória.

## Licença

Uso acadêmico e científico.
