# LifecycleComponent

A LifecycleComponent is a special type of ROS2 node that follows a state-controlled lifecycle, similar to a service managment system such a systemd. Instead of simply starting and running, these components transition between predefined states, allowing for more robust and controlled managment.

# Why Lifecycle is needed in robotics

Problems with traditional nodes:
* Disorderly initialization of resources
* Difficulty recovering from failures
* No standard way to pause/resume
* Uncontrolled startup/shutdown order
* Difficulty testing and debugging

Advantages of Lifecycle:
* Controlled initialization-configuration before activation
* Failure recovery-reversible transitions
* Resource managment - Orderly release
* Testing - predictible states for testing
* Critial systems - Precise control over operations

# 

```
UNCONFIGURED → CONFIGURING → INACTIVE → ACTIVATING → ACTIVE
      ↑            ↓            ↑           ↓           ↓
      └────────────┴────────────┴───────────┴───────────┘
                   TRANSICIONES REVERSIBLES
```

Tabla de Estados y Significado:
|Estado	| Descripción |	Permite Publicar? |	Permite Procesar?  |
|-------|-------------|--------------------|------------------------|
|UNCONFIGURED   |	Estado inicial, sin configuración	|❌ No	|❌ No
|CONFIGURING    |   Configurando recursos	            |❌ No	|❌ No
|INACTIVE	    |   Configurado pero inactivo	        |❌ No	|❌ No
|ACTIVATING	    |   Activando procesamiento	            |❌ No	|❌ No
|ACTIVE	        |   Totalmente operativo	            |✅ Sí	|✅ Sí
|DEACTIVATING	|   Desactivando procesamiento	        |❌ No	|❌ No
|CLEANING UP	|   Limpiando recursos	                |❌ No	|❌ No
|SHUTTING DOWN	|   Apagándose	                        |❌ No	|❌ No
|FINALIZED	    |   Finalizado	                        |❌ No	|❌ No

```
configure()     : UNCONFIGURED → CONFIGURING → INACTIVE
activate()      : INACTIVE → ACTIVATING → ACTIVE
deactivate()    : ACTIVE → DEACTIVATING → INACTIVE
cleanup()       : INACTIVE → CLEANING UP → UNCONFIGURED
shutdown()      : Cualquier estado → SHUTTING DOWN → FINALIZED
```


# LifecycleComponent implementation in C++

## Basic structure

see ..