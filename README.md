# PETER_SIMULATION
Simulación de P.E.T.E.R para paper en el semillero de neurocontrol motor.
## SHORTCUTS

[Link compartido del documento](https://uao-my.sharepoint.com/:f:/g/personal/samuel_carlos_uao_edu_co/EsC_FuYnnO5Jhq126P5lIN4BZYnXlXEZ-dV7QUh0XY8A0w?e=DaVQGd) 

[Link de bitácoras semanales](https://uao-my.sharepoint.com/:f:/g/personal/samuel_carlos_uao_edu_co/Erp2ENIspZxNgMopqcnKfAUBfoev0AsDrE33obHUKmgpDg?e=dBNilw) 

## ¿Cómo usar el repositorio?

### VIDEO TUTORIAL

 <a href="https://www.youtube.com/watch?v=xyDA1aFvYTs">
  <img src="Titulo.png" alt="Texto alternativo" width="700"/>
</a>


### Configuración Inicial

- **Clonar el repositorio**: Este comando copiará los archivos del repositorio en el pc:
    ```bash
    git clone git@github.com:sammcar/PETER_SIMULATION.git
    ```
    Si te aparece "Permission Denied (Publickey)", tienes que seguir el tutorial para usar SSH 

### En caso de hacer cambios en el equipo local (hacer un commit)

1. **Ir a la carpeta raiz para asegurar de hacer un commit de todo**: 
    ```bash
    cd PETER_SIMULATION
    ```
    
2. **Actualizar el contenido**: 
    ```bash
    git pull origin main
    ```
    
3. **Verificar los archivos que creaste/modificaste**:
    ```bash
    git status
    ```
    
4. **Añadir los archivos que creaste/modificaste**:
    ```bash
    git add .
    ```
    
5. **Hacer el commit**:
    ```bash
    git commit -m "<mensaje>"
    ```
    
6. **Hacer el push**:
    ```bash
    git push origin main
    ```


### Solución para problemas de autenticación sin SSH

Si enfrentas problemas de autenticación, puedes ejecutar los siguientes comandos para asegurarte de estar utilizando la autenticación SSH:

1. Establecer la URL remota:
   ```bash
   git remote set-url origin git@github.com:DiegoACaro/Unity_ws.git
   ```

2. Agregar el origen remoto (si es necesario):
   ```bash
   git remote add origin git@github.com:DiegoACaro/Unity_ws.git
   ```
- Asegúrate de haber configurado tu clave SSH correctamente. Sigue la [documentación oficial de GitHub](https://docs.github.com/es/authentication/connecting-to-github-with-ssh/about-ssh) para más detalles.
---



## Uso de ramas (Branches) para trabajar en paralelo:

Para trabajar de forma óptima y organizada en el Paper, se vuelve necesario el uso de las ramas:

### 1. Creación de una nueva rama

Ya creé una rama con el nombre de cada integrante del grupo, aún así, si quieres crear una nueva debes usar el siguiente comando:

```bash
git branch <nombre de la rama>
```

Ahora, para cambiar a tu rama de desarrollo deberás usar el siguiente comando(En este caso `nombre de la rama` será tu nombre):

```bash
git switch <nombre de la rama>
```

Ejemplo:

```bash
git switch dieguito
```

### 2. Hacer cambios y subirlos

Una vez que hayas hecho un cambio en tu rama, lo siguiente será subir los cambios que hiciste (en tu rama) al repositorio:
Nota: Los cambios se harán únicamente en tu rama, no en la rama principal del repositorio

**Añadir los cambios y hacer un commit**:
   ```bash
   git add .
   git commit -m "Descripción del cambio"
   ```
**Subir los cambios a tu rama del repositorio**:
   ```bash
   git push origin <nombre-rama>
   ```
