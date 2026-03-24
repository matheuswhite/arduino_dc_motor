// Indica para o Rust que não vamos utilizar o std (partes do OS + Alocação dinâmica)
#![no_std]
// Indica para o Rust que não usaremos a main tradicional
#![no_main]

// Imports do HAL do Arduino, da lib Aule, do core do Rust, panic_hal e ufmt.
use arduino_hal::{
    adc::Channel,
    clock::MHz16,
    hal::{port::PH6, usart::Usart0},
    port::{mode::PwmOutput, Pin},
    simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm},
    Adc, Peripherals,
};
use aule::prelude::*;
use core::time::Duration;
use panic_halt as _;
use ufmt::uwrite;

// Indica o ponto de entrada do firmware (ou a função main).
#[arduino_hal::entry]
fn main() -> ! {
    // Obtendo os periféricos do Arduino, como ADC, PWM e etc.
    let dp = arduino_hal::Peripherals::take().unwrap();

    // Execução do PID e malha fechada, passando os valores de Kp, Ki e Kd.
    closed_loop(dp, 1.0, 0.0, 0.0);
}

fn closed_loop(dp: Peripherals, kp: f64, ki: f64, kd: f64) -> ! {
    // Obtendo os pinos do Arduino, por exemplo: a0, d9, etc.
    let pins = arduino_hal::pins!(dp);

    // Constroi um objeto do tipo ADC, passando o periférico ADC do arduino.
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    // Configura o pino A0, como uma entrada analógica.
    let a0 = pins.a0.into_analog_input(&mut adc).into_channel();
    // Constroi um objeto do tipo Sensor, para ler a tensão de entrada.
    let mut sensor = Sensor::new(adc, a0);

    // Constroi um objeto do tipo Timer2Pwm, passando o periférico de Timer para o Arduino.
    let timer2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    // Configura o pino D9, como uma saída digital, com a função de PWM.
    let pwm_output = pins.d9.into_output().into_pwm(&timer2);
    // Constroi um objeto do tipo Actuator, para enviar uma tensão de saída.
    let mut actuator = Actuator::new(pwm_output);

    // Constroi um objeto Sinusoid, para servir como um sinal de referencia senoidal, com amplitude 2V, com frequencia de 0.5Hz e fase 0.
    let mut reference = Sinusoid::new(2.0, Duration::from_secs_f32(0.5), 0.0);
    // Offset DC da referencia. Desta forma, agora a senoide de referencia varia entre 1V e 5V.
    let reference_offset = 3.0;
    // Constroi um objeto PID, para servir como o nosso controlador.
    let mut controller = PID::new(kp, ki, kd);
    // Constroi um objeto EndlessTime, para servir como o motor de execução. Essa variável vai gerar o tempo de execução, que será usado pelos nossos objetos.
    let execution_engine = EndlessTime::new(0.03);

    // Configura a Serial do Arduino, para conseguirmos enviar dados via print
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // Loop de execução. Note que o motor de execução vai gerar a variável `time`, que contém o tempo de amostragem e o tempo total da simulação.
    for time in execution_engine {
        // Constroi o sinal vindo do sensor. Esse sinal contém a tensão lida pelo sensor, além dos parametros de tempo contidos no time.
        let sensor_signal = time * sensor.as_block();
        // Constroi o sinal de referencia.
        let ref_signal = time * reference.as_block() + reference_offset;
        // Calcula o erro, sendo a referencia menos o sinal lido do sensor.
        let error = ref_signal - sensor_signal;

        // Obtém o sinal de controle (output), usando o erro como entrada do controlador.
        let output = error * controller.as_block();
        // Envia o sinal de controle para atuador.
        let _ = output * actuator.as_block();

        // Imprime na serial o valor da referencia.
        print_float(&mut serial, ref_signal.value);
        // Imprime na serial o valor do sinal lido pelo sensor.
        print_float(&mut serial, sensor_signal.value);
        // Imprime um `\n` para o monitor do arduino entender cada um dos valores anteriores como um ponto de uma curva que será apresentado no gráfico. Desta forma, teremos 2 pontos, um para cada curva.
        let _ = uwrite!(&mut serial, "\n");

        // Espera o tempo de amostragem descrito dentro da variável `time`
        arduino_hal::delay_ms(time.delta.dt().as_millis() as u32);
    }

    // Macro para indicar que esse ponto nunca vai ser atingido, pois o for não vai encerrar, pois o tipo é EndlessTime. Ou seja, a execução executa sem fim (endless).
    unreachable!();
}

// Função para imprimir números em ponto flutuante.
fn print_float(serial: &mut Usart0<MHz16>, value: f64) {
    // Obtem a parte inteira, convertendo o float para int.
    let integer_part = value as i32;
    // Obtém a parte fracionária, removendo a parte inteira e multiplicando por 1000. Após isso, convertemos para int. Desta forma, teremos a precisão de 3 casas decimais.
    let fract_part = ((integer_part as f64 - value) * 1000.0) as i32;
    // Imprime a parte inteira e a parte fracionária na serial do Arduino.
    let _ = uwrite!(serial, "{}.{}", integer_part, fract_part);
}

// Define a estrutura Sensor, para facilitar a leitura do ADC e embutir a conversão do valor do ADC em tensão.
pub struct Sensor {
    // Guarda o periférico do ADC
    adc: Adc,
    // Guarda o pino analógico usado para o ADC (também chamado de canal do ADC).
    channel: Channel,
}

// Função executada em tempo de compilação para calcular um dividor de tensão.
const fn voltage_divider(vin: f64, r1: f64, r2: f64) -> f64 {
    (r2 / (r1 + r2)) * vin
}

// Função executada em tempo de compilação para calcular o valor máximo de um ADC, dada uma resolução em bits.
const fn adc_max_value(resolution_bits: u32) -> f64 {
    2i32.pow(resolution_bits) as f64
}

// Métodos da estrutura Sensor.
impl Sensor {
    // Indica a tensão máxima que pode ser lida pelo ADC.
    const MAX_SENSOR_VOLTAGE: f64 = voltage_divider(12.0, 10_000.0, 5_000.0);
    // Indica o valor máximo do ADC de 10-bits de resolução.
    const ADC_MAX_VALUE: f64 = adc_max_value(10);

    // Constroi um novo objeto do tipo Sensor. É necessário passar o periférico do ADC e qual canal (ou pino analogico) será utilizado.
    pub fn new(adc: Adc, channel: Channel) -> Self {
        Self { adc, channel }
    }
}

// Implementa o trait Block da lib Aule para a estrutura Sensor. Esse trait permite usar o tipo `Signal` como entrada e saída.
impl Block for Sensor {
    // Define que a entrada será o tipo vazio, pois o sensor não recebe nada como entrada.
    type Input = ();
    // Define que a saída será um float de 64-bits, pois o sensor envia uma tensão como saída.
    type Output = f64;

    // Calcula a saída do Sensor, a cada entrada.
    fn output(&mut self, input: Signal<Self::Input>) -> Signal<Self::Output> {
        // Lê o valor do ADC, como um inteiro sem sinal de 10-bits.
        let adc_value = self.adc.read_blocking(&self.channel);
        // Converte o valor do ADC em uma tensão.
        let voltage = (adc_value as f64 / Self::ADC_MAX_VALUE) * Self::MAX_SENSOR_VOLTAGE;

        // Converte o tipo de entrada de vazio em float de 64-bits. Além disso, coloca o valor de `voltage` no sinal.
        input.map(|_| voltage)
    }
}

// Define a estrutura Actuator, para facilitar o envio de tensão para o motor e converter esse valor em um sinal PWM.
pub struct Actuator {
    // Guarda o periférico do PWM.
    pwm: Pin<PwmOutput<Timer2Pwm>, PH6>,
}

// Métodos da estrutura Actuator.
impl Actuator {
    // Indica a tensão máxima que o atuador pode enviar. Esse valor é correspondente a 255 do PWM (ou 100% do PWM).
    const MAX_ACTUATOR_VOLTAGE: f64 = 12.0;

    // Constroi um novo objeto do tipo Actuator. É necessário passar o periférico do PWM.
    pub fn new(pwm: Pin<PwmOutput<Timer2Pwm>, PH6>) -> Self {
        Self { pwm }
    }
}

// Implementa o trait Block da lib Aule para a estrutura Actuator.
impl Block for Actuator {
    // Define que a entrada será um float de 64-bits, pois o atuador precisa de um valor de tensão para ser convertido em porcentagem PWM e enviado.
    type Input = f64;
    // Define que a saída será o tipo vazio, pois o atuador não retorna nada de saída.
    type Output = ();

    // Recebe a tensão como entrada e envia via PWM.
    fn output(&mut self, input: Signal<Self::Input>) -> Signal<Self::Output> {
        // Limita a tensão de entrada entre 0V até 12V.
        let input_value = input.value.clamp(0.0, Actuator::MAX_ACTUATOR_VOLTAGE);
        // Converte o valor de tensão em ciclo de trabalho do PWM. Os limites de valores do PWM é de 0 até 255 (100%).
        let duty = ((input_value * 255.0) / Actuator::MAX_ACTUATOR_VOLTAGE).clamp(0.0, 255.0) as u8;

        // Envia o valor do ciclo de trabalho via PWM
        self.pwm.set_duty(duty);
        // Habilita o PWM
        self.pwm.enable();

        // Converte o tipo de entrada de float de 64-bits em tipo vazio.
        input.map(|_| ())
    }
}
