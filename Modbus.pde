//#include <Modbus.h>
//void initModbusSlave()   //include in Setup() loop
//{
//        /* Modbus setup example, the master must use the same COM parameters */
//        /* 115200 bps, 8N1, two-device network */
//        regs[MB_REG0]=123;
//        regs[MB_REG1]=456;
//        init_mb_slave(115200, 'n', 0);
//}
//
//
//void Modbus() 
//{
//    /* This is all for the Modbus slave */
//    start_mb_slave(MB_SLAVE, regs, MB_REGS);
//
//    if (written.num_regs) {
//        switch (written.num_regs) {
//        case REGISTER_ONE:
//          break;
//        
//        
//        written.num_regs=0;
//    }
//}
