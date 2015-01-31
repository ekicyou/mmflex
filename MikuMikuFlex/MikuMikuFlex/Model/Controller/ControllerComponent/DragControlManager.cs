using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMF.Model.Controller.ControllerComponent
{
    class DragControlManager
    {
        private bool lastState;

        private bool lastMouseState;

        private bool isDragging;

        private Point lastPoint;

        private ILockableController locker;

        public DragControlManager(ILockableController locker)
        {
            this.locker = locker;
        }

        public bool IsDragging
        {
            get
            {
                return isDragging;
            }
        }

        public bool checkNeedHighlight(bool result)
        {
            return isDragging || (result && !locker.IsLocked);
        }

        public Vector2 Delta { get; private set; }

        public void checkBegin(bool result,bool mouseState,Point mousePosition)
        {
            if (result && lastState && !lastMouseState && mouseState && !isDragging && !locker.IsLocked)
            {
                locker.IsLocked = true;
                isDragging = true;
            }
            Delta = new Vector2(mousePosition.X - lastPoint.X, mousePosition.Y - lastPoint.Y);
        }

        public void checkEnd(bool result,bool mouseState,Point mousePosition)
        {
            if (!mouseState && isDragging)
            {
                locker.IsLocked = false;
                isDragging = false;
            }

            lastState = result;
            lastMouseState = mouseState;
            lastPoint = mousePosition;
        }


    }
}
