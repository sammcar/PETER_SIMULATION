# PETER_SIMULATION
Simulación de P.E.T.E.R para paper en el semillero de neurocontrol motor.
## SHORTCUTS

[Link compartido del documento](https://uao-my.sharepoint.com/:f:/g/personal/samuel_carlos_uao_edu_co/EsC_FuYnnO5Jhq126P5lIN4BZYnXlXEZ-dV7QUh0XY8A0w?e=DaVQGd) 

[Link de bitácoras semanales](https://uao-my.sharepoint.com/:f:/g/personal/samuel_carlos_uao_edu_co/Erp2ENIspZxNgMopqcnKfAUBfoev0AsDrE33obHUKmgpDg?e=dBNilw) 

## ¿Cómo usar el repositorio?

###VIDEO TUTORIAL
 <a href="https://youtu.be/OxkJlgVeY2w">
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
    git commit -m "<mensaje>" .
    ```
    
6. **Hacer el push**:
    ```bash
    git push origin main .
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
