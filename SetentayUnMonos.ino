/*
 Name:		SetentayUnMonos.ino
 Created:	03/09/2024 10:53:28
 Author:	Ofel
*/

#include <TMCStepper_UTILITY.h>
//#include <ArticulatedTriangle2D.h>
//#include <Point2D.h>
//#include <Point3D.h>
#include <TMCStepper.h>
#include <ArduinoSTL.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <vector>
#include <queue>

#include <math.h>

struct Nodo {
	int fila, columna;
	std::vector<Nodo> camino;
};

struct Coordenadas {
	double x;
	double y;
};

struct Angles {
	double theta1;
	double theta2;
};

struct Punto {
	double x;
	double y;
};

const int TAMANO_TABLERO = 8;
char tablero[TAMANO_TABLERO][TAMANO_TABLERO];

bool esMovimientoValidoCaballo(int filaInicio, int columnaInicio, int filaFin, int columnaFin) {
	int dx = abs(filaFin - filaInicio);
	int dy = abs(columnaFin - columnaInicio);
	return (dx == 2 && dy == 1) || (dx == 1 && dy == 2);
}

bool esPosicionLibre(int fila, int columna) {
	return tablero[fila][columna] == ' ';
}


void inicializarTablero() {
	for (int i = 0; i < TAMANO_TABLERO; ++i) {
		for (int j = 0; j < TAMANO_TABLERO; ++j) {
			tablero[i][j] = ' '; // Espacio vac�o indica que la celda est� libre
		}
	}
}

//ArticulatedTriangle2D trig(100, 100, true);

const double L1 = 100.0; // Longitud del primer segmento
const double L2 = 100.0; // Longitud del segundo segmento

// Configuración del motor del hombro (NEMA17 con TMC2209)
constexpr auto STEP_PIN = 6;
constexpr auto DIR_PIN = 3;
constexpr auto ENABLE_PIN = 51;
constexpr auto R_SENSE = 0.11f;      // R_SENSE para c�lculo de corriente
constexpr auto DRIVER_ADDRESS = 0b00;       // Direcci�n del driver TMC2209 seg�n MS1 y MS2
#define SERIAL_PORT Serial3

// Configuración del motor del codo (24BYJ48 con ULN2003)
constexpr auto HALFSTEP = 8;
constexpr auto motorPin1 = 8;     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
constexpr auto motorPin2 = 9;     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
constexpr auto motorPin3 = 10;    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
constexpr auto motorPin4 = 11;    // IN4 on ULN2003 ==> Orange on 28BYJ-48

constexpr auto LEVA_HOMBRO_PIN = 5;
constexpr auto LEVA_CODO_PIN = 4;

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper hombro(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper codo(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
MultiStepper motores;

double posicionActualX = 0.0;
double posicionActualY = 0.0;
double anguloActualHombro = 0.0; // �ngulo actual del hombro
double anguloActualCodo = 0.0;   // �ngulo actual del codo


long pasosMotores[2]; // Array para almacenar las pasosMotores objetivo

Coordenadas calcularCoordenadasDesdeCentro(String comando) {
	int columna = comando[0] - 'a'; // Columna [a-h]
	int fila = comando[1] - '1';    // Fila [1-8]
	double x = (columna - 3.5) * 40; // Ajuste para centrar en el tablero
	double y = (fila - 3.5) * 40;    // Ajuste para centrar en el tablero
	return { x, y };
}

std::vector<Punto> bresenham(double x0, double y0, double x1, double y1) {
	std::vector<Punto> puntos;
	int dx = abs(x1 - x0);
	int dy = abs(y1 - y0);
	int sx = (x0 < x1) ? 1 : -1;
	int sy = (y0 < y1) ? 1 : -1;
	int err = dx - dy;

	while (true) {
		puntos.push_back({ x0, y0 });
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}
	return puntos;
}
//.............................
void resolverCinematicaInversa(Punto punto, double& anguloHombro1, double& anguloCodo1, double& anguloHombro2, double& anguloCodo2) {
	double x = punto.x;
	double y = punto.y;

	// Calcular la distancia desde el origen hasta el punto objetivo
	double distancia = sqrt(x * x + y * y);

	// Verificar si el punto está al alcance del brazo
	if (distancia > (L1 + L2) || distancia < fabs(L1 - L2)) {
		Serial.println("Error: El punto está fuera del alcance del brazo.");
		return;
	}

	// Calcular el ángulo del hombro para ambas configuraciones
	double cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
	double sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2);

	// Configuración del codo arriba
	anguloCodo1 = atan2(sinTheta2, cosTheta2);
	anguloHombro1 = atan2(y, x) - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);

	// Configuración del codo abajo
	anguloCodo2 = atan2(-sinTheta2, cosTheta2);
	anguloHombro2 = atan2(y, x) - atan2(L2 * -sinTheta2, L1 + L2 * cosTheta2);

	// Convertir los ángulos a grados
	anguloHombro1 = degrees(anguloHombro1);
	anguloCodo1 = degrees(anguloCodo1);
	anguloHombro2 = degrees(anguloHombro2);
	anguloCodo2 = degrees(anguloCodo2);
}


void ajustarCoordenadas(Punto& punto, double& anguloActualHombro, double& anguloActualCodo, double posicionActualX, double posicionActualY) {
	double anguloHombro1, anguloCodo1, anguloHombro2, anguloCodo2;

	// Resolver la cinemática inversa para obtener los ángulos
	resolverCinematicaInversa(punto, anguloHombro1, anguloCodo1, anguloHombro2, anguloCodo2);

	// Imprimir las soluciones calculadas
	Serial.print("Solución 1 - Hombro: "); Serial.print(anguloHombro1); Serial.print(", Codo: "); Serial.println(anguloCodo1);
	Serial.print("Solución 2 - Hombro: "); Serial.print(anguloHombro2); Serial.print(", Codo: "); Serial.println(anguloCodo2);

	// Calcular la diferencia entre las soluciones y la posición actual
	double diferencia1 = abs(anguloHombro1 - anguloActualHombro) + abs(anguloCodo1 - anguloActualCodo);
	double diferencia2 = abs(anguloHombro2 - anguloActualHombro) + abs(anguloCodo2 - anguloActualCodo);

	// Imprimir las diferencias calculadas
	Serial.print("Diferencia 1: "); Serial.println(diferencia1);
	Serial.print("Diferencia 2: "); Serial.println(diferencia2);

	// Elegir la solución más cercana a la posición actual
	if (diferencia1 < diferencia2) {
		anguloActualHombro = -anguloHombro1; // Invertir el signo
		anguloActualCodo = anguloCodo1;
		Serial.println("Elegida Solución 1.");
	}
	else {
		anguloActualHombro = -anguloHombro2; // Invertir el signo
		anguloActualCodo = anguloCodo2;
		Serial.println("Elegida Solución 2.");
	}

	// Actualizar las coordenadas del punto
	punto.x = posicionActualX;
	punto.y = posicionActualY;

	// Imprimir los ángulos finales seleccionados
	Serial.print("Ángulo Hombro Final: "); Serial.println(anguloActualHombro);
	Serial.print("Ángulo Codo Final: "); Serial.println(anguloActualCodo);
}


void moverAPosicion(String comando) {
	int columna = comando[0] - 'a'; // Columna [a-h]
	int fila = comando[1] - '1';    // Fila [1-8]
	Serial.print("Moviendo a la posicion "); Serial.print(comando); Serial.println("...");

	// Convertir notación de ajedrez a coordenadas en mm desde el centro del tablero
	Coordenadas coord = calcularCoordenadasDesdeCentro(comando);
	double x = coord.x;
	double y = coord.y;

	// Generar puntos intermedios usando el algoritmo de Bresenham
	std::vector<Punto> puntos = bresenham(posicionActualX, posicionActualY, x, y);

	// Imprimir el contenido del vector puntos
	Serial.println("Puntos generados por Bresenham:");
	for (const auto& punto : puntos) {
		Serial.print("Punto - X: "); Serial.print(punto.x); Serial.print(", Y: "); Serial.println(punto.y);
	}

	// Mover el efector a través de los puntos generados
	for (auto& punto : puntos) {
		// Ajustar las coordenadas para evitar cambios bruscos
		ajustarCoordenadas(punto, anguloActualHombro, anguloActualCodo, posicionActualX, posicionActualY);

		// Mover los motores a la posición ajustada
		moverMotores();

		// Actualizar la posición actual del efector
		posicionActualX = punto.x;
		posicionActualY = punto.y;
	}

	// Actualizar la posición actual del efector
	posicionActualX = x;
	posicionActualY = y;
}
//.............................




double suavizarAngulo(double nuevoAngulo, double anguloActual) {
	double diferencia = nuevoAngulo - anguloActual;
	if (diferencia > 180) {
		nuevoAngulo -= 360;
	}
	else if (diferencia < -180) {
		nuevoAngulo += 360;
	}
	return nuevoAngulo;
}

void realizarHoming() {
	Serial.println("Realizando homing para los motores...");
	// Homing para el codo
	bool estadoLevaCodo = digitalRead(LEVA_CODO_PIN);
	if (estadoLevaCodo == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		codo.setSpeed(-800);
		while (digitalRead(LEVA_CODO_PIN) == LOW) {
			codo.runSpeed();
		}
	}
	else {
		codo.setSpeed(800);
		while (digitalRead(LEVA_CODO_PIN) == HIGH) {
			codo.runSpeed();
		}
	}
	codo.setCurrentPosition(0); // Establece la posici�n actual transitoria como cero

	// Homing para el hombro
	bool estadoLevaHombro = digitalRead(LEVA_HOMBRO_PIN);
	Serial.print("Estado leva hombro: "); Serial.println(estadoLevaHombro);
	bool levaInicial = estadoLevaHombro;
	if (estadoLevaHombro == LOW) {
		// Mover el motor hasta que se active el fin de carrera
		hombro.setSpeed(800);
		while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
			hombro.runSpeed();
		}
	}
	else {
		hombro.setSpeed(-800);
		while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
			hombro.runSpeed();
		}
	}
	hombro.setCurrentPosition(0); // Establece la posici�n actual como cero
	delay(2000);
	// Mover los motores a la configuración de reposo inicial
	pasosMotores[0] = -4300;   //3940  positivo = CCW
	pasosMotores[1] = -460;   //1575  positivo = CW
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();
	// Establece la posici�n actual como cero
	codo.setCurrentPosition(0);
	hombro.setCurrentPosition(0);
	Serial.println("Homing completado.");
}


long calcularPasosHombro(double theta1) {
	double pasosPorGradoHombro = 15450 / 360;
	long pasos = -theta1 * pasosPorGradoHombro;
	return pasos;
}

long calcularPasosCodo(double theta2) {
	double pasosPorGradoCodo = 3964 / 360;
	long pasos = theta2 * pasosPorGradoCodo;
	return pasos;
}

void moverMotores() {
	// Verificar si la solución es válida
	if (isnan(anguloActualHombro) || isnan(anguloActualCodo)) {
		Serial.println("Error: No se pudo calcular los ángulos para la posición objetivo.");
		return;
	}

	// Convertir ángulos a pasos de motor (en pasos)
	pasosMotores[0] = calcularPasosHombro(anguloActualHombro);
	pasosMotores[1] = calcularPasosCodo(anguloActualCodo);
	Serial.print("Pasos: ("); Serial.print(pasosMotores[0]); Serial.print(", "); Serial.print(pasosMotores[1]); Serial.println(")");
	//pasosMotores[0] = pasosMotores[0];
	// 
	// Mover los motores a las posiciones calculadas
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();

	// Actualizar los ángulos actuales
	Serial.print("Ángulo Hombro Actual: "); Serial.println(anguloActualHombro);
	Serial.print("Ángulo Codo Actual: "); Serial.println(anguloActualCodo);
}


void recalcularFactores() {
	// Variables para almacenar los pasos totales
	long pasosTotalesHombro = 0;
	long pasosTotalesCodo = 0;

	// Realizar homing antes de comenzar
	realizarHoming();

	// Calcular pasos para un giro completo del hombro
	hombro.setSpeed(500);
	bool estadoInicialLevaHombro = digitalRead(LEVA_HOMBRO_PIN);
	while (digitalRead(LEVA_HOMBRO_PIN) == estadoInicialLevaHombro) {
		//hombro.move(1); // Mover un paso
		//hombro.runToPosition();
		//pasosTotalesHombro++;
		hombro.runSpeed();
		pasosTotalesHombro = hombro.currentPosition();
	}
	Serial.print("Pasos totales hombro (180 grados): ");
	Serial.println(pasosTotalesHombro);

	// Calcular pasos para un giro completo del codo
	codo.setSpeed(500);
	bool estadoInicialLevaCodo = digitalRead(LEVA_CODO_PIN);
	while (digitalRead(LEVA_CODO_PIN) == estadoInicialLevaCodo) {
		//codo.move(1); // Mover un paso
		//codo.runToPosition();
		//pasosTotalesCodo++;
		codo.runSpeed();
		pasosTotalesCodo = codo.currentPosition();
	}
	Serial.print("Pasos totales codo (180 grados): ");
	Serial.println(pasosTotalesCodo);

	// Calcular los factores de conversi�n
	double pasosPorGradoHombro = pasosTotalesHombro / 180.0;
	double pasosPorGradoCodo = pasosTotalesCodo / 180.0;

	Serial.print("Pasos por grado hombro: ");
	Serial.println(pasosPorGradoHombro);
	Serial.print("Pasos por grado codo: ");
	Serial.println(pasosPorGradoCodo);
}


void setup() {
	Serial.begin(115200);

	SERIAL_PORT.begin(115200); // Configura la comunicación con el TMC2209

	// Configuración del driver TMC2209
	pinMode(ENABLE_PIN, OUTPUT);
	digitalWrite(ENABLE_PIN, LOW); // Habilitar el driver

	driver.begin();
	driver.toff(5);
	driver.rms_current(800); // Corriente en mA para el NEMA17
	driver.microsteps(16);   // Configurar microstepping
	driver.en_spreadCycle(false); // Deshabilitar spreadCycle, usa StealthChop
	driver.pwm_autoscale(true); // Activar auto scale PWM
	driver.enn(); // Habilitar el driver

	// Configuración de los pines
	pinMode(LEVA_HOMBRO_PIN, INPUT_PULLUP);
	pinMode(LEVA_CODO_PIN, INPUT_PULLUP);

	// Configuración inicial de los motores
	hombro.setMaxSpeed(800);
	hombro.setAcceleration(800);
	codo.setMaxSpeed(800);
	codo.setAcceleration(800);

	// Añadir los motores al objeto MultiStepper
	motores.addStepper(hombro); // posición '0'
	motores.addStepper(codo);    // posición '1'

	// Homing: Mueve los motores hasta posicionarse en el origen
	Serial.println("Realizando homing...");
	realizarHoming();
	Serial.println("Homing completado.");
	delay(5000);

	// Ajustar la posición inicial del efector al centro de la casilla d4
	Coordenadas coordInicial = calcularCoordenadasDesdeCentro("d4");
	Serial.print("Coordenadas iniciales: ("); Serial.print(coordInicial.x); Serial.print(", "); Serial.print(coordInicial.y); Serial.println(")");

	//posicionActualX = coordInicial.x;
	//posicionActualY = coordInicial.y;
	posicionActualX = 0.0;
	posicionActualY = 0.0;

	// Mover el efector al centro de la casilla d4
	Punto puntoInicial = { posicionActualX, posicionActualY };
	ajustarCoordenadas(puntoInicial, anguloActualHombro, anguloActualCodo, posicionActualX, posicionActualY);
	moverMotores();

	// Espera comandos del usuario a través del puerto serie
	Serial.println("Homing completado. Listo para recibir comandos.");

	codo.disableOutputs();
	hombro.disableOutputs();
}

void loop() {
	if (Serial.available() > 0) {
		String input = Serial.readStringUntil('\n');
		Serial.print("Comando recibido: "); Serial.println(input);

		moverAPosicion(input);
	}
}

void imprimirPuntos(const std::vector<Punto>& puntos) {
	Serial.println("Contenido del vector puntos:");
	for (const auto& punto : puntos) {
		Serial.print("Punto: (");
		Serial.print(punto.x);
		Serial.print(", ");
		Serial.print(punto.y);
		Serial.println(")");
	}
}






